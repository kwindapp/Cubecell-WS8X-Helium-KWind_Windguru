#include "ws8x.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

// =============================================
// CONSTANTS
// =============================================

static const int LORA_PAYLOAD_SIZE = 18;

// =============================================
// STATIC VARIABLES
// =============================================

static float windSpeeds[MAX_READINGS];
static float windDirs[MAX_READINGS];
static unsigned long windTimes[MAX_READINGS];

static int readingCount = 0;
static int writeIndex = 0;

static float currentWindDir = 0.0f;
static float currentWindSpeed = 0.0f;

static float batVoltageF = 0.0f;
static float humidityF = 0.0f;
static float temperatureF = 0.0f;

static unsigned long lastReadTime = 0;
static unsigned long firstReadTime = 0;

static bool forceZeroMode = false;
static int noWindCount = 0;

// =============================================
// HELPER FUNCTIONS
// =============================================

static int latestReadingIndex()
{
    if (readingCount <= 0) {
        return -1;
    }

    return (writeIndex - 1 + MAX_READINGS) % MAX_READINGS;
}

static bool isNoWindCondition(const String& line)
{
#if NO_WIND_DETECTION_ENABLED == 0
    (void)line;
    return false;
#else
    int eqPos = line.indexOf('=');

    if (eqPos < 0) {
        return false;
    }

    String value = line.substring(eqPos + 1);
    value.trim();

    if (line.indexOf("x_y =") >= 0) {
        int xyValue = value.toInt();

        if (xyValue <= X_Y_THRESHOLD) {
#if PRINT_NO_WIND == 1
            Serial.printf(
                "[NO-WIND] x_y = %d -> possible no wind\n",
                xyValue
            );
#endif
            return true;
        }
    }

    if (line.indexOf("Get_Cali_Ult_X") >= 0) {
        int calibrationValue = value.toInt();

        if (calibrationValue == CALI_NO_WIND_VALUE) {
#if PRINT_NO_WIND == 1
            Serial.printf(
                "[NO-WIND] Get_Cali_Ult_X = %d -> possible no wind\n",
                calibrationValue
            );
#endif
            return true;
        }
    }

    return false;
#endif
}

static float applyNoiseFilter(float rawSpeed)
{
#if NOISE_FILTER_ENABLED == 0
    return rawSpeed;
#else
    if (
        rawSpeed < WIND_SPEED_THRESHOLD ||
        rawSpeed > WIND_SPIKE_THRESHOLD
    ) {
#if PRINT_FILTER == 1
        Serial.printf(
            "[FILTER] Speed %.1f m/s -> zero\n",
            rawSpeed
        );
#endif
        return 0.0f;
    }

    return rawSpeed;
#endif
}

static float calculateWindSpeedAverage(
    const float* speeds,
    int count
)
{
    if (count <= 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    int validCount = 0;

    for (int i = 0; i < count; i++) {
        if (speeds[i] >= 0.0f && speeds[i] < 100.0f) {
            sum += speeds[i];
            validCount++;
        }
    }

    if (validCount == 0) {
        return 0.0f;
    }

    return sum / validCount;
}

static float calculateWindDirectionAverage(
    const float* directions,
    const float* speeds,
    int count
)
{
    if (count <= 0) {
        return currentWindDir;
    }

    float sinSum = 0.0f;
    float cosSum = 0.0f;
    int validCount = 0;

    for (int i = 0; i < count; i++) {
        if (
            speeds[i] > MIN_WIND_SPEED_FOR_DIR &&
            directions[i] >= 0.0f &&
            directions[i] < 360.0f
        ) {
            float radians = directions[i] * PI / 180.0f;

            sinSum += sin(radians);
            cosSum += cos(radians);
            validCount++;
        }
    }

    /*
     * If fewer than three usable direction readings exist,
     * return the newest valid direction.
     */
    if (validCount < 3) {
        for (int step = 1; step <= count; step++) {
            int index =
                (writeIndex - step + MAX_READINGS) %
                MAX_READINGS;

            if (
                directions[index] >= 0.0f &&
                directions[index] < 360.0f
            ) {
                return directions[index];
            }
        }

        return currentWindDir;
    }

    float averageDegrees =
        atan2(sinSum, cosSum) * 180.0f / PI;

    if (averageDegrees < 0.0f) {
        averageDegrees += 360.0f;
    }

    return averageDegrees;
}

static float calculateMaxGust(
    const float* speeds,
    int count
)
{
    float maximum = 0.0f;

    for (int i = 0; i < count; i++) {
        if (
            speeds[i] >= 0.0f &&
            speeds[i] < 100.0f &&
            speeds[i] > maximum
        ) {
            maximum = speeds[i];
        }
    }

    return maximum;
}

static float calculateMinLull(
    const float* speeds,
    int count
)
{
    float minimum = 999.0f;
    int validCount = 0;

    for (int i = 0; i < count; i++) {
        if (speeds[i] >= 0.0f && speeds[i] < 100.0f) {
            if (speeds[i] < minimum) {
                minimum = speeds[i];
            }

            validCount++;
        }
    }

    return validCount > 0 ? minimum : 0.0f;
}

static void storeWindReading(
    float speed,
    float direction,
    unsigned long timestamp
)
{
    windSpeeds[writeIndex] = speed;
    windDirs[writeIndex] = direction;
    windTimes[writeIndex] = timestamp;

    writeIndex =
        (writeIndex + 1) %
        MAX_READINGS;

    if (readingCount < MAX_READINGS) {
        readingCount++;
    }

    if (firstReadTime == 0) {
        firstReadTime = timestamp;
    }

#if PRINT_COLLECTION == 1
    Serial.printf(
        "[STORED] %d/%d: wind=%.1f m/s, direction=%.0f deg\n",
        readingCount,
        MAX_READINGS,
        speed,
        direction
    );
#endif
}

// =============================================
// SERIAL PARSING
// =============================================

static void processSerialLine(String line)
{
    line.trim();

    if (line.length() == 0) {
        return;
    }

    unsigned long currentTime = millis();
    lastReadTime = currentTime;

#if PRINT_RAW_SERIAL == 1
    Serial.print("[RAW] ");
    Serial.println(line);
#endif

    // -----------------------------------------
    // Wind direction
    // Example: direction = 333
    // -----------------------------------------

    if (line.indexOf("direction =") >= 0) {
        int eqPos = line.indexOf('=');

        if (eqPos >= 0) {
            String value = line.substring(eqPos + 1);
            value.trim();

            float direction = value.toFloat();

            if (
                direction >= 0.0f &&
                direction < 360.0f
            ) {
                currentWindDir = direction;

#if PRINT_PARSED_DATA == 1
                Serial.printf(
                    "[PARSED] direction = %.0f deg\n",
                    currentWindDir
                );
#endif
            }
        }

        return;
    }

    // -----------------------------------------
    // Wind speed
    // Example: wind = 3
    // -----------------------------------------

    if (line.indexOf("wind =") >= 0) {
        int eqPos = line.indexOf('=');

        if (eqPos >= 0) {
            String value = line.substring(eqPos + 1);
            value.trim();

            float rawSpeed = value.toFloat();

            if (
                rawSpeed >= 0.0f &&
                rawSpeed < 100.0f
            ) {
                float filteredSpeed =
                    applyNoiseFilter(rawSpeed);

                if (forceZeroMode) {
                    filteredSpeed = 0.0f;
                }

                currentWindSpeed = filteredSpeed;

#if PRINT_PARSED_DATA == 1
                Serial.printf(
                    "[PARSED] wind raw=%.1f, filtered=%.1f m/s\n",
                    rawSpeed,
                    filteredSpeed
                );
#endif

                storeWindReading(
                    filteredSpeed,
                    currentWindDir,
                    currentTime
                );
            }
        }

        return;
    }

    // -----------------------------------------
    // Temperature
    // Example: Temperature = 26.3
    // -----------------------------------------

    if (
        line.indexOf("Temperature") >= 0 &&
        line.indexOf('=') >= 0
    ) {
        int eqPos = line.indexOf('=');
        String value = line.substring(eqPos + 1);
        value.trim();

        if (value.endsWith("C")) {
            value.remove(value.length() - 1);
            value.trim();
        }

        float temperature = value.toFloat();

        if (
            temperature > -50.0f &&
            temperature < 100.0f
        ) {
            temperatureF = temperature;

#if PRINT_PARSED_DATA == 1
            Serial.printf(
                "[PARSED] temperature = %.1f C\n",
                temperatureF
            );
#endif
        }

        return;
    }

    // -----------------------------------------
    // Humidity
    // Example: Humi = 63%
    // -----------------------------------------

    if (
        line.indexOf("Humi") >= 0 &&
        line.indexOf('=') >= 0
    ) {
        int eqPos = line.indexOf('=');
        String value = line.substring(eqPos + 1);
        value.trim();

        if (value.endsWith("%")) {
            value.remove(value.length() - 1);
            value.trim();
        }

        float humidity = value.toFloat();

        if (
            humidity >= 0.0f &&
            humidity <= 100.0f
        ) {
            humidityF = humidity;

#if PRINT_PARSED_DATA == 1
            Serial.printf(
                "[PARSED] humidity = %.1f %%\n",
                humidityF
            );
#endif
        }

        return;
    }

    // -----------------------------------------
    // WS80 battery
    // Example: BatVoltage = 3.26V
    // -----------------------------------------

    if (
        line.indexOf("BatVoltage") >= 0 &&
        line.indexOf('=') >= 0
    ) {
        int eqPos = line.indexOf('=');
        String value = line.substring(eqPos + 1);
        value.trim();

        if (value.endsWith("V")) {
            value.remove(value.length() - 1);
            value.trim();
        }

        float voltage = value.toFloat();

        if (
            voltage > 0.0f &&
            voltage < 10.0f
        ) {
            batVoltageF = voltage;

#if PRINT_PARSED_DATA == 1
            Serial.printf(
                "[PARSED] WS80 battery = %.2f V\n",
                batVoltageF
            );
#endif
        }

        return;
    }

    // -----------------------------------------
    // No-wind detection
    // -----------------------------------------

    if (
        line.indexOf("x_y =") >= 0 ||
        line.indexOf("Get_Cali_Ult_X") >= 0
    ) {
        if (isNoWindCondition(line)) {
            noWindCount++;

#if PRINT_NO_WIND == 1
            Serial.printf(
                "[NO-WIND] count = %d/%d\n",
                noWindCount,
                NO_WIND_CONFIRM_COUNT
            );
#endif

            if (noWindCount >= NO_WIND_CONFIRM_COUNT) {
                forceZeroMode = true;

#if PRINT_NO_WIND == 1
                Serial.println(
                    "[NO-WIND] Force-zero mode active"
                );
#endif
            }
        }

        return;
    }
}

// =============================================
// PUBLIC FUNCTIONS
// =============================================

void ws8x_init()
{
    /*
     * CubeCell AB01 / AB01 V2 external hardware UART.
     *
     * WS80 TX  -> CubeCell UART_RX
     * WS80 GND -> CubeCell GND
     * CubeCell UART_TX remains physically disconnected.
     *
     * CubeCell framework 1.6.0 needs the named UART pins
     * passed explicitly. UART_TX is configured by the driver
     * but does not need a physical connection for receive-only.
     */
    Serial1.begin(
        115200,
        SERIAL_8N1,
        UART_RX,
        UART_TX
    );

    Serial1.setTimeout(100);

    for (int i = 0; i < MAX_READINGS; i++) {
        windSpeeds[i] = 0.0f;
        windDirs[i] = 0.0f;
        windTimes[i] = 0;
    }

    readingCount = 0;
    writeIndex = 0;

    currentWindDir = 0.0f;
    currentWindSpeed = 0.0f;

    batVoltageF = 0.0f;
    humidityF = 0.0f;
    temperatureF = 0.0f;

    lastReadTime = 0;
    firstReadTime = 0;

    forceZeroMode = false;
    noWindCount = 0;

    Serial.println();
    Serial.println("==========================================");
    Serial.println("[WS8X] INITIALIZED");
    Serial.println("[WS8X] CubeCell AB01 external Serial1");
    Serial.println("[WS8X] RX pin = UART_RX (connect WS80 TX)");
    Serial.println("[WS8X] TX pin = UART_TX (leave disconnected)");
    Serial.println("[WS8X] Baud rate: 115200");

    Serial.printf(
        "[WS8X] Required readings: %d\n",
        MIN_READINGS
    );

    Serial.println("==========================================");
}

void ws8x_checkSerial()
{
    while (Serial1.available() > 0) {
        String line = Serial1.readStringUntil('\n');
        processSerialLine(line);
    }

#if PRINT_COLLECTION == 1
    static unsigned long lastStatusPrint = 0;

    if (
        readingCount > 0 &&
        millis() - lastStatusPrint >= 5000UL
    ) {
        lastStatusPrint = millis();

        Serial.printf(
            "[COLLECT] %d/%d readings collected\n",
            readingCount,
            MIN_READINGS
        );
    }
#endif
}

bool ws8x_has_valid_data()
{
    unsigned long currentTime = millis();

    if (readingCount == 0) {
        static unsigned long lastNoDataPrint = 0;

        if (
            currentTime - lastNoDataPrint >=
            30000UL
        ) {
            lastNoDataPrint = currentTime;

            Serial.println(
                "[COLLECT] No WS80 data received"
            );
        }

        return false;
    }

    int latestIndex = latestReadingIndex();

    if (latestIndex < 0) {
        return false;
    }

    /*
     * Do not upload old data when the WS80 has stopped
     * sending serial readings.
     */
    if (
        currentTime - windTimes[latestIndex] >
        DATA_TIMEOUT_MS
    ) {
        Serial.println(
            "[COLLECT] Data is stale; payload blocked"
        );

        return false;
    }

    if (readingCount < MIN_READINGS) {
#if PRINT_COLLECTION == 1
        static int lastPrintedCount = -1;

        if (
            readingCount == 1 ||
            readingCount % 5 == 0
        ) {
            if (readingCount != lastPrintedCount) {
                lastPrintedCount = readingCount;

                unsigned long collectionTime =
                    firstReadTime > 0
                        ? currentTime - firstReadTime
                        : 0;

                Serial.printf(
                    "[COLLECT] %d/%d readings, %.1f seconds\n",
                    readingCount,
                    MIN_READINGS,
                    collectionTime / 1000.0f
                );
            }
        }
#endif

        return false;
    }

#if PRINT_COLLECTION == 1
    unsigned long collectionTime =
        firstReadTime > 0
            ? currentTime - firstReadTime
            : 0;

    Serial.printf(
        "[COLLECT] READY: %d readings in %.1f seconds\n",
        readingCount,
        collectionTime / 1000.0f
    );
#endif

    return true;
}

float ws8x_get_wind_direction()
{
    return calculateWindDirectionAverage(
        windDirs,
        windSpeeds,
        readingCount
    );
}

float ws8x_get_wind_speed()
{
    return calculateWindSpeedAverage(
        windSpeeds,
        readingCount
    );
}

float ws8x_get_wind_gust()
{
    return calculateMaxGust(
        windSpeeds,
        readingCount
    );
}

float ws8x_get_temperature()
{
    return temperatureF;
}

float ws8x_get_battery_voltage()
{
    return batVoltageF;
}

float ws8x_get_humidity()
{
    return humidityF;
}

bool ws8x_force_zero_mode()
{
#if NO_WIND_DETECTION_ENABLED == 0
    return false;
#else
    return forceZeroMode;
#endif
}

void ws8x_reset_no_wind_counter()
{
    noWindCount = 0;
    forceZeroMode = false;

#if PRINT_NO_WIND == 1
    Serial.println(
        "[NO-WIND] Counter and force-zero mode reset"
    );
#endif
}

void ws8x_print_data()
{
    float averageSpeed =
        ws8x_get_wind_speed();

    float averageDirection =
        ws8x_get_wind_direction();

    float gust =
        ws8x_get_wind_gust();

    float lull =
        calculateMinLull(
            windSpeeds,
            readingCount
        );

    unsigned long collectionTime =
        firstReadTime > 0
            ? millis() - firstReadTime
            : 0;

    Serial.println();
    Serial.println("==========================================");
    Serial.println("WS80 WEATHER DATA");
    Serial.println("------------------------------------------");

    Serial.printf(
        "Readings:         %d/%d\n",
        readingCount,
        MAX_READINGS
    );

    Serial.printf(
        "Collection time:  %.1f seconds\n",
        collectionTime / 1000.0f
    );

    Serial.printf(
        "Force zero mode:  %s\n",
        forceZeroMode ? "ACTIVE" : "OFF"
    );

    Serial.printf(
        "Wind average:     %.1f m/s (%.1f knots)\n",
        averageSpeed,
        averageSpeed * 1.94384f
    );

    Serial.printf(
        "Wind gust:        %.1f m/s (%.1f knots)\n",
        gust,
        gust * 1.94384f
    );

    Serial.printf(
        "Wind lull:        %.1f m/s\n",
        lull
    );

    Serial.printf(
        "Wind direction:   %.0f degrees\n",
        averageDirection
    );

    Serial.printf(
        "Temperature:      %.1f C\n",
        temperatureF
    );

    Serial.printf(
        "Humidity:         %.1f %%\n",
        humidityF
    );

    Serial.printf(
        "WS80 battery:     %.2f V\n",
        batVoltageF
    );

    Serial.println("==========================================");
    Serial.println();
}

void ws8x_populate_lora_buffer(
    uint8_t* buffer,
    int size
)
{
    if (buffer == NULL) {
        Serial.println(
            "[WS8X] ERROR: Payload buffer is NULL"
        );

        return;
    }

    /*
     * This payload contains nine 16-bit values:
     * 9 x 2 bytes = 18 bytes.
     */
    if (size < LORA_PAYLOAD_SIZE) {
        Serial.printf(
            "[WS8X] ERROR: Payload buffer needs %d bytes; received %d\n",
            LORA_PAYLOAD_SIZE,
            size
        );

        if (size > 0) {
            memset(buffer, 0, size);
        }

        return;
    }

    memset(buffer, 0, size);

    if (!ws8x_has_valid_data()) {
        Serial.println(
            "[WS8X] WARNING: Payload not created"
        );

        return;
    }

    float averageSpeed =
        ws8x_get_wind_speed();

    float averageDirection =
        ws8x_get_wind_direction();

    float gust =
        ws8x_get_wind_gust();

    float lull =
        calculateMinLull(
            windSpeeds,
            readingCount
        );

    int16_t payloadDirection =
        (int16_t)round(averageDirection);

    int16_t payloadAverage =
        (int16_t)round(averageSpeed * 10.0f);

    int16_t payloadGust =
        (int16_t)round(gust * 10.0f);

    int16_t payloadLull =
        (int16_t)round(lull * 10.0f);

    int16_t payloadSensorBattery =
        (int16_t)round(batVoltageF * 100.0f);

    int16_t payloadHumidity =
        (int16_t)round(humidityF * 100.0f);

    int16_t payloadTemperature =
        (int16_t)round(temperatureF * 10.0f);

    uint16_t payloadRain = 0;

    /*
     * This currently contains the WS80 battery voltage
     * in millivolts, matching your existing payload.
     */
    uint16_t payloadVoltageMillivolts =
        (uint16_t)round(batVoltageF * 1000.0f);

    int offset = 0;

    memcpy(
        &buffer[offset],
        &payloadDirection,
        sizeof(payloadDirection)
    );
    offset += sizeof(payloadDirection);

    memcpy(
        &buffer[offset],
        &payloadAverage,
        sizeof(payloadAverage)
    );
    offset += sizeof(payloadAverage);

    memcpy(
        &buffer[offset],
        &payloadGust,
        sizeof(payloadGust)
    );
    offset += sizeof(payloadGust);

    memcpy(
        &buffer[offset],
        &payloadLull,
        sizeof(payloadLull)
    );
    offset += sizeof(payloadLull);

    memcpy(
        &buffer[offset],
        &payloadSensorBattery,
        sizeof(payloadSensorBattery)
    );
    offset += sizeof(payloadSensorBattery);

    memcpy(
        &buffer[offset],
        &payloadHumidity,
        sizeof(payloadHumidity)
    );
    offset += sizeof(payloadHumidity);

    memcpy(
        &buffer[offset],
        &payloadTemperature,
        sizeof(payloadTemperature)
    );
    offset += sizeof(payloadTemperature);

    memcpy(
        &buffer[offset],
        &payloadRain,
        sizeof(payloadRain)
    );
    offset += sizeof(payloadRain);

    memcpy(
        &buffer[offset],
        &payloadVoltageMillivolts,
        sizeof(payloadVoltageMillivolts)
    );
    offset += sizeof(payloadVoltageMillivolts);

    ws8x_print_data();

    Serial.printf(
        "[PAYLOAD] Size: %d bytes\n",
        offset
    );

    Serial.print("[PAYLOAD] HEX: ");

    for (int i = 0; i < offset; i++) {
        Serial.printf("%02X", buffer[i]);
    }

    Serial.println();
}

void ws8x_reset_counters()
{
    float lastSpeed = currentWindSpeed;
    float lastDirection = currentWindDir;

    for (int i = 0; i < MAX_READINGS; i++) {
        windSpeeds[i] = 0.0f;
        windDirs[i] = 0.0f;
        windTimes[i] = 0;
    }

    readingCount = 0;
    writeIndex = 0;

    firstReadTime = 0;
    lastReadTime = 0;

    noWindCount = 0;
    forceZeroMode = false;

    currentWindSpeed = lastSpeed;
    currentWindDir = lastDirection;

    Serial.println(
        "[WS8X] Counters reset; collecting next interval"
    );
}
