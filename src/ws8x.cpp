#include "ws8x.h"
#include <Arduino.h>
#include <math.h>

// Variables for wind data
static float currentWindDir = 0;    // Store current direction
static float currentWindSpeed = 0;  // Store current speed
static float currentGust = 0;       // Store current gust
static float currentLull = 0;       // Store current lull
static float currentTemp = 0;       // Store current temperature

// Variables for other metrics
static float batVoltageF = 0;
static float capVoltageF = 0;      // WS85: Capacitor Voltage, WS80: Humidity
static float temperatureF = 0;
static float rain = 0;
static int rainSum = 0;

// Store raw readings for better averaging
#define MAX_READINGS 60
static float windSpeeds[MAX_READINGS];
static float windDirs[MAX_READINGS];
static int readingCount = 0;

// Gateway matching settings
#define WIND_AVG_PERIOD 60  // 60 seconds
#define MIN_WIND_SPEED_FOR_DIR 0.5  // Ignore direction below this speed (m/s)

void ws8x_init()
{
    // Initialize arrays
    memset(windSpeeds, 0, sizeof(windSpeeds));
    memset(windDirs, 0, sizeof(windDirs));
    readingCount = 0;
}

void ws8x_checkSerial()
{
    const int maxIterations = 100;
    int iterationCount = 0;
    
    while (Serial.available() > 0 && iterationCount < maxIterations)
    {
        iterationCount++;
        String line = Serial.readStringUntil('\n');
        line.trim();
        
#ifdef PRINT_WX_SERIAL
        Serial.println(line);
#endif
        
        if (line.length() > 0)
        {
            int index = line.indexOf('=');
            if (index != -1)
            {
                String key = line.substring(0, index);
                String value = line.substring(index + 1);
                key.trim();
                value.trim();

                if (value.endsWith("V"))
                {
                    value = value.substring(0, value.length() - 1);
                }

                // Store current values as they arrive
                if (key == "WindDir")
                {
                    currentWindDir = value.toFloat();
                    if (readingCount < MAX_READINGS) {
                        windDirs[readingCount] = currentWindDir;
                    }
                }
                else if (key == "WindSpeed")
                {
                    currentWindSpeed = value.toFloat();
                    if (readingCount < MAX_READINGS) {
                        windSpeeds[readingCount] = currentWindSpeed;
                        readingCount++;
                    }
                }
                else if (key == "WindGust")
                {
                    currentGust = value.toFloat();
                }
                else if (key == "BatVoltage")
                {
                    batVoltageF = value.toFloat();
                }
                else if (key == "CapVoltage")
                {
                    capVoltageF = value.toFloat();
                }
                else if (key == "Humi")
                {
                    capVoltageF = value.toFloat();
                }
                else if (key == "GXTS04Temp" || key == "Temperature")
                {
                    if (value != "--")
                    {
                        temperatureF = value.toFloat();
                        currentTemp = temperatureF;
                    }
                }
                else if (key == "Rain")
                {
                    rain = value.toFloat();
                }
            }
        }
    }

    if (iterationCount >= maxIterations)
    {
        Serial.println("Maximum serial reading iterations reached");
    }
}

// Calculate wind direction using circular median (more stable than vector average)
float calculateWindDirection(float* directions, int count, float* speeds) {
    if (count == 0) return 0;
    
    // Only calculate direction if wind speed is above threshold
    float avgSpeed = 0;
    int speedCount = 0;
    for (int i = 0; i < count; i++) {
        if (speeds[i] > MIN_WIND_SPEED_FOR_DIR) {
            avgSpeed += speeds[i];
            speedCount++;
        }
    }
    
    if (speedCount == 0) return 0;  // No significant wind
    
    // Use a simpler approach: most common direction range
    // Group directions into 10-degree bins
    int bins[36] = {0};
    int maxBin = 0;
    int maxBinIndex = 0;
    
    for (int i = 0; i < count; i++) {
        if (speeds[i] > MIN_WIND_SPEED_FOR_DIR) {
            int bin = (int)(directions[i] / 10) % 36;
            bins[bin]++;
            if (bins[bin] > maxBin) {
                maxBin = bins[bin];
                maxBinIndex = bin;
            }
        }
    }
    
    // Return the center of the most common bin
    float result = (maxBinIndex * 10) + 5;
    if (result >= 360) result -= 360;
    
    return result;
}

float calculateWindSpeedAverage(float* speeds, int count) {
    if (count == 0) return 0;
    
    float sum = 0;
    for (int i = 0; i < count; i++) {
        sum += speeds[i];
    }
    return sum / count;
}

void ws8x_populate_lora_buffer(uint8_t* m_lora_app_data, int size)
{
    uint16_t deviceVoltage_mv = getBatteryVoltage();
    Serial.printf("Battery voltage : %d mV\n\r", deviceVoltage_mv);
    
    // Calculate averages using stored readings
    float velAvg = calculateWindSpeedAverage(windSpeeds, readingCount);
    
    // Calculate direction using improved method
    float dirAvg = calculateWindDirection(windDirs, readingCount, windSpeeds);
    
    // Use current gust (already the maximum from the gateway)
    float gust = currentGust;
    float lull = 0;
    
    // Find actual lull from stored readings
    if (readingCount > 0) {
        lull = windSpeeds[0];
        for (int i = 1; i < readingCount; i++) {
            if (windSpeeds[i] < lull) {
                lull = windSpeeds[i];
            }
        }
    }

    // Print data for debugging
    Serial.printf("Wind Speed Avg: %.1f m/s, Wind Dir Avg: %d°, Gust: %.1f m/s, Lull: %.1f m/s\n",
                  velAvg, (int)dirAvg, gust, lull);
    Serial.printf("Battery Voltage: %.1f V, Capacitor/Humidity: %.1f, Temperature: %.1f °C\n",
                  batVoltageF, capVoltageF, temperatureF);
    Serial.printf("Rain: %.1f mm, Device mv : %d, Readings: %d\n", rain, deviceVoltage_mv, readingCount);

    // Populate the buffer
    memset(m_lora_app_data, 0, size);

    // Round the values (matching gateway precision)
    float roundedVelAvg = round(velAvg * 10) / 10.0;
    float roundedDirAvg = round(dirAvg);                     // Integer degrees
    float roundedGust = round(gust * 10) / 10.0;
    float roundedLull = round(lull * 10) / 10.0;
    float roundedBatVoltageF = round(batVoltageF * 10) / 10.0;
    float roundedCapVoltageF = round(capVoltageF * 10) / 10.0;
    float roundedTemperatureF = round(temperatureF * 10) / 10.0;
    float roundedRain = round(rain * 10) / 10.0;

    // Convert values to integers with proper scaling
    int16_t intDirAvg = (int16_t)roundedDirAvg;              // 0-359
    int16_t intVelAvg = (int16_t)(roundedVelAvg * 10);       // 1 decimal
    int16_t intGust = (int16_t)(roundedGust * 10);           // 1 decimal
    int16_t intLull = (int16_t)(roundedLull * 10);           // 1 decimal
    int16_t intBatVoltageF = (int16_t)(roundedBatVoltageF * 100); // 2 decimals
    int16_t intCapVoltageF = (int16_t)(roundedCapVoltageF * 100); // 2 decimals
    int16_t intTemperatureF = (int16_t)(roundedTemperatureF * 10); // 1 decimal
    uint16_t intRain = (uint16_t)(roundedRain * 10);         // 1 decimal
    
    // Pack the integers into the buffer
    int offset = 0;
    memcpy(&m_lora_app_data[offset], &intDirAvg, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data[offset], &intVelAvg, sizeof(int16_t));
    offset += sizeof(int16_t);
    
#ifndef SEND_MIN_BYTES
    Serial.println("populating full buffer of 18 bytes");
    memcpy(&m_lora_app_data[offset], &intGust, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data[offset], &intLull, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data[offset], &intBatVoltageF, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data[offset], &intCapVoltageF, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data[offset], &intTemperatureF, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data[offset], &intRain, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy(&m_lora_app_data[offset], &deviceVoltage_mv, sizeof(uint16_t));
    offset += sizeof(uint16_t);
#endif

    // Print debug information
    Serial.print("Payload bytes: ");
    for (int i = 0; i < offset; i++)
    {
        Serial.printf("%02X", m_lora_app_data[i]);
    }
    Serial.println();
}

void ws8x_reset_counters() {
    // Reset stored readings
    memset(windSpeeds, 0, sizeof(windSpeeds));
    memset(windDirs, 0, sizeof(windDirs));
    readingCount = 0;
    
    // Reset current values
    currentWindDir = 0;
    currentWindSpeed = 0;
    currentGust = 0;
    currentLull = 0;
    currentTemp = 0;
    
    // Reset other metrics
    batVoltageF = 0;
    capVoltageF = 0;
    temperatureF = 0;
    rain = 0;
    rainSum = 0;
}