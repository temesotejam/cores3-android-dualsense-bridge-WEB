/* CoreS3 Wi-Fi controller bridge: DualSense -> Android -> WebSocket -> CoreS3. */
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "control_config.h"

#define MAX_WS_FRAME 192

extern const uint8_t control_page_html_start[] asm("_binary_src_control_page_html_start");
extern const uint8_t control_page_html_end[] asm("_binary_src_control_page_html_end");
extern void debug_logf(const char *format, ...);
extern void screen_statusf(const char *format, ...);
extern void outputs_setup(void);
extern void outputs_apply(int leftWing, int rightWing, int vectorServo, int motorSpeed);
extern bool outputs_get_servo_limits(servo_limit_settings_t *limits);
extern bool outputs_set_servo_limits(const servo_limit_settings_t *limits, bool persist);

static const char *TAG = "cores3_control";

typedef struct {
    int8_t left_wing;     /* left stick X: -100 ... +100 */
    int8_t right_wing;    /* right stick X: -100 ... +100 */
    int8_t left_servo;    /* L2: -100 center ... 0 left */
    int8_t right_servo;   /* R2: 0 center ... +100 right */
    uint8_t motor_speed;  /* D-pad up/down adjusted speed: 0 ... 100 */
    bool enabled;         /* physical dead-man switch: L1 */
    bool emergency_stop;  /* PS button or phone emergency-stop button */
    bool dpad_up;
    bool dpad_down;
} motion_command_t;

static motion_command_t current_command;
static int64_t last_command_us;
static int64_t last_speed_step_us;
static bool stop_latched = true;
static bool control_armed;
static uint8_t target_motor_speed;
static int8_t current_vector_servo;

#define MOTOR_SPEED_STEP 5
#define MOTOR_SPEED_REPEAT_MS 120

static int clamp_int(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void apply_motion_command(const motion_command_t *command)
{
    int target_vector = 0;
    bool has_left = command->left_servo > CONTROL_VECTOR_SERVO_DEADBAND;
    bool has_right = command->right_servo > CONTROL_VECTOR_SERVO_DEADBAND;

    if (has_left && has_right) {
        const int left_candidate = -command->left_servo;
        const int right_candidate = command->right_servo;
        const int left_delta = abs(left_candidate - current_vector_servo);
        const int right_delta = abs(right_candidate - current_vector_servo);
        target_vector = left_delta <= right_delta ? left_candidate : right_candidate;
    } else if (has_left) {
        target_vector = -command->left_servo;
    } else if (has_right) {
        target_vector = command->right_servo;
    }

    const int delta = target_vector - current_vector_servo;
    if (delta > CONTROL_VECTOR_SERVO_MAX_STEP) {
        current_vector_servo += CONTROL_VECTOR_SERVO_MAX_STEP;
    } else if (delta < -CONTROL_VECTOR_SERVO_MAX_STEP) {
        current_vector_servo -= CONTROL_VECTOR_SERVO_MAX_STEP;
    } else {
        current_vector_servo = (int8_t)target_vector;
    }

    outputs_apply(command->left_wing, command->right_wing,
                  current_vector_servo, command->motor_speed);
}

static void stop_motion(void)
{
    const motion_command_t stop = {0};
    if (current_command.enabled || current_command.left_wing || current_command.right_wing ||
        current_command.left_servo || current_command.right_servo || current_command.motor_speed) {
        ESP_LOGW(TAG, "Motion command stopped");
    }
    current_command = stop;
    target_motor_speed = 0;
    last_speed_step_us = 0;
    current_vector_servo = 0;
    apply_motion_command(&stop);
}

static bool is_neutral_command(const motion_command_t *command)
{
    return !command->emergency_stop &&
           !command->enabled &&
           !command->dpad_up &&
           !command->dpad_down &&
           command->left_wing == 0 &&
           command->right_wing == 0 &&
           command->left_servo == 0 &&
           command->right_servo == 0 &&
           command->motor_speed == 0;
}

/* Packet format: C,sequence,LX,LY,RX,RY,L2,R2,L1,DUP,DDOWN,ESTOP */
static bool parse_command(const char *payload, motion_command_t *command)
{
    uint32_t sequence;
    int lx, ly, rx, ry, l2, r2, l1, dpad_up, dpad_down, estop;
    int fields = sscanf(payload, "C,%" SCNu32 ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                        &sequence, &lx, &ly, &rx, &ry, &l2, &r2, &l1, &dpad_up, &dpad_down, &estop);
    (void)sequence;
    if (fields != 11 ||
        (l1 != 0 && l1 != 1) ||
        (dpad_up != 0 && dpad_up != 1) ||
        (dpad_down != 0 && dpad_down != 1) ||
        (estop != 0 && estop != 1)) {
        return false;
    }

    memset(command, 0, sizeof(*command));
    command->left_wing = (int8_t)clamp_int(lx, -100, 100);
    command->right_wing = (int8_t)clamp_int(rx, -100, 100);
    command->left_servo = (int8_t)clamp_int(l2, 0, 100);
    command->right_servo = (int8_t)clamp_int(r2, 0, 100);
    command->emergency_stop = estop != 0;
    command->dpad_up = dpad_up != 0;
    command->dpad_down = dpad_down != 0;

    /* E-stop requires L1 to be released before a command can be re-armed. */
    if (command->emergency_stop) stop_latched = true;
    if (l1 == 0) stop_latched = false;
    command->enabled = l1 != 0 && !stop_latched;

    int64_t now_us = esp_timer_get_time();
    bool speed_repeat_ready = last_speed_step_us == 0 ||
                              now_us - last_speed_step_us >= (int64_t)MOTOR_SPEED_REPEAT_MS * 1000;
    if (!command->emergency_stop && command->enabled &&
        speed_repeat_ready && command->dpad_up != command->dpad_down) {
        if (command->dpad_up) {
            target_motor_speed = (uint8_t)clamp_int(target_motor_speed + MOTOR_SPEED_STEP, 0, 100);
        } else if (command->dpad_down) {
            target_motor_speed = (uint8_t)clamp_int((int)target_motor_speed - MOTOR_SPEED_STEP, 0, 100);
        }
        last_speed_step_us = now_us;
    }
    command->motor_speed = target_motor_speed;

    if (!command->enabled) {
        target_motor_speed = 0;
        last_speed_step_us = 0;
        command->left_wing = 0;
        command->right_wing = 0;
        command->left_servo = 0;
        command->right_servo = 0;
        command->motor_speed = 0;
    }
    return true;
}

/* Packet format:
 * R,sequence,a0,a1,a2,a3,a4,a5,a6,a7,b0,b1,b2,b3,b4,b5,b6,b7,b12,b13,b16
 * Values are normalized to -100..100 for axes and 0..100 for buttons.
 */
static void log_raw_report(const char *payload)
{
    uint32_t sequence;
    int values[20];
    int fields = sscanf(payload,
                        "R,%" SCNu32 ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                        &sequence,
                        &values[0], &values[1], &values[2], &values[3], &values[4],
                        &values[5], &values[6], &values[7], &values[8], &values[9],
                        &values[10], &values[11], &values[12], &values[13], &values[14],
                        &values[15], &values[16], &values[17], &values[18], &values[19]);
    if (fields != 21) {
        ESP_LOGW(TAG, "Malformed raw report: %s", payload);
        debug_logf("Malformed raw report: %s", payload);
        screen_statusf("Malformed raw report");
        return;
    }

    ESP_LOGI(TAG,
             "RAW seq=%" PRIu32
             " axes[A0..A7]=[%d,%d,%d,%d,%d,%d,%d,%d] "
             "buttons[B0,B1,B2,B3,B4,B5,B6,B7,B12,B13,B16,RES]=[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
             sequence,
             values[0], values[1], values[2], values[3],
             values[4], values[5], values[6], values[7],
             values[8], values[9], values[10], values[11],
             values[12], values[13], values[14], values[15],
             values[16], values[17], values[18], values[19]);
    debug_logf("RAW seq=%" PRIu32
               " A=[%d,%d,%d,%d,%d,%d,%d,%d] B=[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
               sequence,
               values[0], values[1], values[2], values[3],
               values[4], values[5], values[6], values[7],
               values[8], values[9], values[10], values[11],
               values[12], values[13], values[14], values[15],
               values[16], values[17], values[18], values[19]);
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)control_page_html_start,
                           control_page_html_end - control_page_html_start);
}

static esp_err_t servo_limits_get_handler(httpd_req_t *req)
{
    servo_limit_settings_t limits;
    if (!outputs_get_servo_limits(&limits)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "limits unavailable");
    }

    char response[160];
    snprintf(response, sizeof(response),
             "{\"leftWingMin\":%d,\"leftWingMax\":%d,"
             "\"rightWingMin\":%d,\"rightWingMax\":%d,"
             "\"vectorMin\":%d,\"vectorMax\":%d}",
             limits.left_wing_min, limits.left_wing_max,
             limits.right_wing_min, limits.right_wing_max,
             limits.vector_min, limits.vector_max);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, response);
}

static bool parse_limit_value(const char *body, const char *key, int *out)
{
    char value[16];
    if (httpd_query_key_value(body, key, value, sizeof(value)) != ESP_OK) {
        return false;
    }
    *out = atoi(value);
    return true;
}

static esp_err_t servo_limits_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len >= MAX_WS_FRAME) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
    }

    char body[MAX_WS_FRAME];
    int received = httpd_req_recv(req, body, req->content_len);
    if (received <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body read failed");
    }
    body[received] = '\0';

    servo_limit_settings_t limits;
    int left_min, left_max, right_min, right_max, vector_min, vector_max;
    if (!parse_limit_value(body, "leftWingMin", &left_min) ||
        !parse_limit_value(body, "leftWingMax", &left_max) ||
        !parse_limit_value(body, "rightWingMin", &right_min) ||
        !parse_limit_value(body, "rightWingMax", &right_max) ||
        !parse_limit_value(body, "vectorMin", &vector_min) ||
        !parse_limit_value(body, "vectorMax", &vector_max)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing fields");
    }

    limits.left_wing_min = (int8_t)left_min;
    limits.left_wing_max = (int8_t)left_max;
    limits.right_wing_min = (int8_t)right_min;
    limits.right_wing_max = (int8_t)right_max;
    limits.vector_min = (int8_t)vector_min;
    limits.vector_max = (int8_t)vector_max;

    if (!outputs_set_servo_limits(&limits, true)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid limits");
    }

    stop_motion();
    screen_statusf("Servo limits saved");
    return servo_limits_get_handler(req);
}

static esp_err_t websocket_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Phone control session connected");
        debug_logf("Phone control session connected");
        control_armed = false;
        stop_motion();
        screen_statusf("Phone connected");
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK || frame.len == 0 || frame.len >= MAX_WS_FRAME) {
        stop_motion();
        return ESP_FAIL;
    }

    char payload[MAX_WS_FRAME];
    frame.payload = (uint8_t *)payload;
    err = httpd_ws_recv_frame(req, &frame, sizeof(payload));
    if (err != ESP_OK) {
        stop_motion();
        return err;
    }
    payload[frame.len] = '\0';

    if (payload[0] == 'R') {
        log_raw_report(payload);
        return ESP_OK;
    }

    motion_command_t command;
    if (!parse_command(payload, &command)) {
        ESP_LOGW(TAG, "Rejected malformed control command");
        debug_logf("Rejected malformed control command: %s", payload);
        screen_statusf("Malformed command");
        stop_motion();
        return ESP_FAIL;
    }
    last_command_us = esp_timer_get_time();

    if (!control_armed) {
        if (!is_neutral_command(&command)) {
            stop_motion();
            screen_statusf("Waiting neutral...");
            return ESP_OK;
        }
        control_armed = true;
        screen_statusf("Control armed");
    }

    current_command = command;
    apply_motion_command(&current_command);
    return ESP_OK;
}

static void start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 2;
    config.lru_purge_enable = true;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    const httpd_uri_t index = {.uri = "/", .method = HTTP_GET, .handler = index_handler};
    const httpd_uri_t ws = {.uri = "/ws", .method = HTTP_GET, .handler = websocket_handler,
                            .is_websocket = true};
    const httpd_uri_t servo_limits_get = {
        .uri = "/api/servo-limits", .method = HTTP_GET, .handler = servo_limits_get_handler};
    const httpd_uri_t servo_limits_post = {
        .uri = "/api/servo-limits", .method = HTTP_POST, .handler = servo_limits_post_handler};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ws));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &servo_limits_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &servo_limits_post));
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Phone joined Wi-Fi access point");
        screen_statusf("Wi-Fi client joined");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGW(TAG, "Phone left Wi-Fi access point");
        control_armed = false;
        screen_statusf("Wi-Fi client left");
        stop_motion();
    }
}

static void start_wifi_access_point(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                         wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.ap.ssid, CONTROL_WIFI_SSID, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, CONTROL_WIFI_PASSWORD, sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.ssid_len = strlen(CONTROL_WIFI_SSID);
    wifi_config.ap.channel = CONTROL_WIFI_CHANNEL;
    wifi_config.ap.max_connection = 1;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi ready: SSID=%s; open http://192.168.4.1", CONTROL_WIFI_SSID);
    debug_logf("Wi-Fi ready: SSID=%s", CONTROL_WIFI_SSID);
    screen_statusf("Wi-Fi: %s", CONTROL_WIFI_SSID);
}

void core_setup(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    screen_statusf("NVS ready");
    stop_motion();
    screen_statusf("Starting Wi-Fi...");
    start_wifi_access_point();
    screen_statusf("Starting web...");
    start_web_server();
    screen_statusf("Starting outputs...");
    outputs_setup();
    screen_statusf("Control ready");

    while (true) {
        int64_t elapsed = esp_timer_get_time() - last_command_us;
        if (last_command_us == 0 || elapsed > (int64_t)CONTROL_TIMEOUT_MS * 1000) {
            stop_motion();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
