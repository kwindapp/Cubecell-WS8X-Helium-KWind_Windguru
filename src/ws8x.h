#ifndef WS8X_H
#define WS8X_H

#include <Arduino.h>

// =============================================
// CONFIGURATION
// =============================================

#define GUST_WINDOW_MS         3000UL
#define AVG_WINDOW_MS         60000UL
#define MAX_READINGS           60
#define MIN_READINGS           30
#define MIN_WIND_SPEED_FOR_DIR 0.0f
#define DATA_TIMEOUT_MS        60000UL

// =============================================
// NOISE FILTER SETTINGS
// =============================================

#define WIND_SPEED_THRESHOLD   3.0f
#define WIND_SPIKE_THRESHOLD   8.0f
#define NOISE_FILTER_ENABLED   1

// =============================================
// NO-WIND DETECTION
// =============================================

#define NO_WIND_DETECTION_ENABLED 1
#define X_Y_THRESHOLD             10
#define CALI_NO_WIND_VALUE        58000
#define NO_WIND_CONFIRM_COUNT     3

// =============================================
// DEBUG FLAGS
// =============================================

#define PRINT_RAW_SERIAL       1
#define PRINT_PARSED_DATA      1
#define PRINT_WS80_BLOCKS      0
#define PRINT_COLLECTION       1
#define PRINT_FILTER           1
#define PRINT_NO_WIND          1

// =============================================
// CUBECELL AB01 / AB01 V2 UART
// =============================================

// Receive-only connection:
//
// WS80 TX  -> CubeCell UART_RX
// WS80 GND -> CubeCell GND
//
// CubeCell UART_TX remains disconnected.

#define WS80_RX_PIN UART_RX

// =============================================
// FUNCTION DECLARATIONS
// =============================================

void ws8x_init();
void ws8x_checkSerial();

bool ws8x_has_valid_data();
bool ws8x_force_zero_mode();

float ws8x_get_wind_direction();
float ws8x_get_wind_speed();
float ws8x_get_wind_gust();
float ws8x_get_temperature();
float ws8x_get_battery_voltage();
float ws8x_get_humidity();

void ws8x_print_data();
void ws8x_populate_lora_buffer(uint8_t* buffer, int size);
void ws8x_reset_counters();

#endif // WS8X_H