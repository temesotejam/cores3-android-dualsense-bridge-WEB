#include <Arduino.h>
#include <HardwareSerial.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <Wire.h>
#include <stdarg.h>
#include <array>

#include "control_config.h"

extern "C" void core_setup(void);
extern "C" void debug_logf(const char *format, ...);
extern "C" void outputs_setup(void);
extern "C" void outputs_apply(int leftWing, int rightWing, int vectorServo, int motorSpeed);
extern "C" void screen_statusf(const char *format, ...);

namespace {

constexpr uint8_t kPcaMode1 = 0x00;
constexpr uint8_t kPcaPrescale = 0xfe;
constexpr uint8_t kPcaLed0OnL = 0x06;
constexpr uint8_t kVescCommSetDuty = 5;

HardwareSerial VescSerial(1);
std::array<String, 10> gScreenLines;
size_t gScreenCount = 0;
bool gOutputsInitialized = false;
Preferences gPreferences;
servo_limit_settings_t gServoLimits = {-100, 100, -100, 100, -100, 100};

void redrawScreen()
{
    M5.Display.startWrite();
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(4, 4);
    M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.println("CoreS3 Control");
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    for (size_t i = 0; i < gScreenCount; ++i) {
        M5.Display.println(gScreenLines[i]);
    }
    M5.Display.endWrite();
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void pcaWrite8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(CONTROL_PCA9685_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

bool pcaProbe()
{
    Wire.beginTransmission(CONTROL_PCA9685_ADDR);
    return Wire.endTransmission() == 0;
}

void pcaSetPwm(uint8_t channel, uint16_t onCount, uint16_t offCount)
{
    const uint8_t reg = static_cast<uint8_t>(kPcaLed0OnL + 4 * channel);
    Wire.beginTransmission(CONTROL_PCA9685_ADDR);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(onCount & 0xff));
    Wire.write(static_cast<uint8_t>((onCount >> 8) & 0x0f));
    Wire.write(static_cast<uint8_t>(offCount & 0xff));
    Wire.write(static_cast<uint8_t>((offCount >> 8) & 0x0f));
    Wire.endTransmission();
}

int clampServoCommand(int normalized, int minLimit, int maxLimit)
{
    return constrain(normalized, minLimit, maxLimit);
}

uint16_t servoPulseFromNormalized(int normalized)
{
    normalized = constrain(normalized, -100, 100);
    if (normalized >= 0) {
        return static_cast<uint16_t>(
            CONTROL_SERVO_PULSE_MID +
            (CONTROL_SERVO_PULSE_MAX - CONTROL_SERVO_PULSE_MID) * normalized / 100);
    }

    return static_cast<uint16_t>(
        CONTROL_SERVO_PULSE_MID -
        (CONTROL_SERVO_PULSE_MID - CONTROL_SERVO_PULSE_MIN) * (-normalized) / 100);
}

void setServoNormalized(uint8_t channel, int normalized, int minLimit, int maxLimit)
{
    pcaSetPwm(channel, 0, servoPulseFromNormalized(clampServoCommand(normalized, minLimit, maxLimit)));
}

bool isValidServoLimits(const servo_limit_settings_t &limits)
{
    return limits.left_wing_min >= -100 && limits.left_wing_min < limits.left_wing_max && limits.left_wing_max <= 100 &&
           limits.right_wing_min >= -100 && limits.right_wing_min < limits.right_wing_max && limits.right_wing_max <= 100 &&
           limits.vector_min >= -100 && limits.vector_min < limits.vector_max && limits.vector_max <= 100;
}

void loadServoLimits()
{
    servo_limit_settings_t loaded = {
        static_cast<int8_t>(gPreferences.getChar("lw_min", -100)),
        static_cast<int8_t>(gPreferences.getChar("lw_max", 100)),
        static_cast<int8_t>(gPreferences.getChar("rw_min", -100)),
        static_cast<int8_t>(gPreferences.getChar("rw_max", 100)),
        static_cast<int8_t>(gPreferences.getChar("vec_min", -100)),
        static_cast<int8_t>(gPreferences.getChar("vec_max", 100)),
    };

    if (isValidServoLimits(loaded)) {
        gServoLimits = loaded;
    } else {
        gServoLimits = {-100, 100, -100, 100, -100, 100};
    }
}

void saveServoLimits()
{
    gPreferences.putChar("lw_min", gServoLimits.left_wing_min);
    gPreferences.putChar("lw_max", gServoLimits.left_wing_max);
    gPreferences.putChar("rw_min", gServoLimits.right_wing_min);
    gPreferences.putChar("rw_max", gServoLimits.right_wing_max);
    gPreferences.putChar("vec_min", gServoLimits.vector_min);
    gPreferences.putChar("vec_max", gServoLimits.vector_max);
}

void vescSendDuty(float duty)
{
    duty = constrain(duty, 0.0f, 1.0f);
    const int32_t scaledDuty = static_cast<int32_t>(duty * 100000.0f);
    uint8_t payload[5];
    payload[0] = kVescCommSetDuty;
    payload[1] = static_cast<uint8_t>((scaledDuty >> 24) & 0xff);
    payload[2] = static_cast<uint8_t>((scaledDuty >> 16) & 0xff);
    payload[3] = static_cast<uint8_t>((scaledDuty >> 8) & 0xff);
    payload[4] = static_cast<uint8_t>(scaledDuty & 0xff);

    const uint16_t crc = crc16_ccitt(payload, sizeof(payload));
    uint8_t frame[10];
    frame[0] = 2; /* short packet start */
    frame[1] = sizeof(payload);
    memcpy(frame + 2, payload, sizeof(payload));
    frame[7] = static_cast<uint8_t>((crc >> 8) & 0xff);
    frame[8] = static_cast<uint8_t>(crc & 0xff);
    frame[9] = 3; /* stop byte */
    VescSerial.write(frame, sizeof(frame));
}

} // namespace

void debug_logf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.println(buffer);
}

void screen_statusf(const char *format, ...)
{
    char buffer[96];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (gScreenCount < gScreenLines.size()) {
        gScreenLines[gScreenCount++] = String(buffer);
    } else {
        for (size_t i = 1; i < gScreenLines.size(); ++i) {
            gScreenLines[i - 1] = gScreenLines[i];
        }
        gScreenLines[gScreenLines.size() - 1] = String(buffer);
    }
    redrawScreen();
}

void outputs_setup(void)
{
    gPreferences.begin("ctrl-servo", false);
    loadServoLimits();

    Wire.begin(CONTROL_I2C_SDA_PIN, CONTROL_I2C_SCL_PIN);
    if (!pcaProbe()) {
        debug_logf("PCA9685 not found at 0x%02X on SDA=%d SCL=%d",
                   CONTROL_PCA9685_ADDR, CONTROL_I2C_SDA_PIN, CONTROL_I2C_SCL_PIN);
        screen_statusf("PCA9685 not found");
        return;
    }

    pcaWrite8(kPcaMode1, 0x10); /* sleep before prescale */
    pcaWrite8(kPcaPrescale, static_cast<uint8_t>((25000000.0 / (4096.0 * CONTROL_PCA9685_FREQ)) - 1.0 + 0.5));
    pcaWrite8(kPcaMode1, 0x00);
    delay(5);
    pcaWrite8(kPcaMode1, 0xa1); /* auto-increment + restart */

    setServoNormalized(CONTROL_SERVO_LEFT_WING_CH, 0, gServoLimits.left_wing_min, gServoLimits.left_wing_max);
    setServoNormalized(CONTROL_SERVO_RIGHT_WING_CH, 0, gServoLimits.right_wing_min, gServoLimits.right_wing_max);
    setServoNormalized(CONTROL_SERVO_VECTOR_CH, 0, gServoLimits.vector_min, gServoLimits.vector_max);

    VescSerial.begin(CONTROL_VESC_UART_BAUD, SERIAL_8N1,
                     CONTROL_VESC_UART_RX_PIN, CONTROL_VESC_UART_TX_PIN);
    vescSendDuty(0.0f);
    gOutputsInitialized = true;
    debug_logf("Outputs ready: PCA9685@0x%02X, VESC UART %d baud duty_max=%.2f",
               CONTROL_PCA9685_ADDR, CONTROL_VESC_UART_BAUD, CONTROL_VESC_MAX_DUTY);
    screen_statusf("PCA9685 OK @0x%02X", CONTROL_PCA9685_ADDR);
    screen_statusf("VESC duty max %.2f", CONTROL_VESC_MAX_DUTY);
    debug_logf("Servo limits LW[%d,%d] RW[%d,%d] V[%d,%d]",
               gServoLimits.left_wing_min, gServoLimits.left_wing_max,
               gServoLimits.right_wing_min, gServoLimits.right_wing_max,
               gServoLimits.vector_min, gServoLimits.vector_max);
}

void outputs_apply(int leftWing, int rightWing, int vectorServo, int motorSpeed)
{
    if (!gOutputsInitialized) {
        return;
    }

    setServoNormalized(CONTROL_SERVO_LEFT_WING_CH, leftWing,
                       gServoLimits.left_wing_min, gServoLimits.left_wing_max);
    setServoNormalized(CONTROL_SERVO_RIGHT_WING_CH, rightWing,
                       gServoLimits.right_wing_min, gServoLimits.right_wing_max);
    setServoNormalized(CONTROL_SERVO_VECTOR_CH, vectorServo,
                       gServoLimits.vector_min, gServoLimits.vector_max);

    const float duty = CONTROL_VESC_MAX_DUTY *
                       static_cast<float>(constrain(motorSpeed, 0, 100)) / 100.0f;
    vescSendDuty(duty);
}

extern "C" bool outputs_get_servo_limits(servo_limit_settings_t *limits)
{
    if (limits == nullptr) {
        return false;
    }
    *limits = gServoLimits;
    return true;
}

extern "C" bool outputs_set_servo_limits(const servo_limit_settings_t *limits, bool persist)
{
    if (limits == nullptr || !isValidServoLimits(*limits)) {
        return false;
    }

    gServoLimits = *limits;
    if (persist) {
        saveServoLimits();
    }

    if (gOutputsInitialized) {
        setServoNormalized(CONTROL_SERVO_LEFT_WING_CH, 0,
                           gServoLimits.left_wing_min, gServoLimits.left_wing_max);
        setServoNormalized(CONTROL_SERVO_RIGHT_WING_CH, 0,
                           gServoLimits.right_wing_min, gServoLimits.right_wing_max);
        setServoNormalized(CONTROL_SERVO_VECTOR_CH, 0,
                           gServoLimits.vector_min, gServoLimits.vector_max);
    }

    debug_logf("Servo limits updated LW[%d,%d] RW[%d,%d] V[%d,%d]",
               gServoLimits.left_wing_min, gServoLimits.left_wing_max,
               gServoLimits.right_wing_min, gServoLimits.right_wing_max,
               gServoLimits.vector_min, gServoLimits.vector_max);
    return true;
}

void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(1);
    Serial.begin(115200);
    delay(200);
    screen_statusf("Booting...");
    Serial.println("CORES3 DEBUG SERIAL READY");
    screen_statusf("USB serial ready");
    core_setup();
}

void loop()
{
    delay(10);
}
