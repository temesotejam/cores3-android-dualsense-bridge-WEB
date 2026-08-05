#pragma once

/* Change these before field use. WPA2 requires at least eight characters. */
#define CONTROL_WIFI_SSID     "CoreS3-Control"
#define CONTROL_WIFI_PASSWORD "cores3ctrl"
#define CONTROL_WIFI_CHANNEL  1
#define CONTROL_TIMEOUT_MS    250

/* CoreS3 Port A: I2C for PCA9685 */
#define CONTROL_I2C_SDA_PIN   2
#define CONTROL_I2C_SCL_PIN   1
#define CONTROL_PCA9685_ADDR  0x40
#define CONTROL_PCA9685_FREQ  50

/* PCA9685 servo channel assignment */
#define CONTROL_SERVO_LEFT_WING_CH   0
#define CONTROL_SERVO_RIGHT_WING_CH  1
#define CONTROL_SERVO_VECTOR_CH      2

/* Typical 1.0ms..2.0ms pulse range at 50Hz on 12-bit PCA9685 */
#define CONTROL_SERVO_PULSE_MIN  205
#define CONTROL_SERVO_PULSE_MID  307
#define CONTROL_SERVO_PULSE_MAX  410

/* Rate limits in normalized command units (-100..100) */
#define CONTROL_VECTOR_SERVO_DEADBAND  5
#define CONTROL_VECTOR_SERVO_MAX_STEP  12

/* CoreS3 Port C: UART for VESC */
#define CONTROL_VESC_UART_TX_PIN  17
#define CONTROL_VESC_UART_RX_PIN  18
#define CONTROL_VESC_UART_BAUD    115200
#define CONTROL_VESC_MAX_DUTY     1.00f

typedef struct {
    int8_t left_wing_min;
    int8_t left_wing_max;
    int8_t right_wing_min;
    int8_t right_wing_max;
    int8_t vector_min;
    int8_t vector_max;
} servo_limit_settings_t;
