#include "ws8x.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

// =============================================
// CONSTANTS
// =============================================

static const int LORA_PAYLOAD_SIZE = 18;
static const int MIN_CALIBRATED_READINGS = 2;

// =============================================
// CALIBRATED WS80 READINGS
// =============================================

static float windSpeeds[MAX_READINGS];
static float windGusts[MAX_READINGS];
static float windDirs[MAX_READINGS];

static unsigned long windTimes[MAX_READINGS];

static int readingCount = 0;
static int writeIndex = 0;

// =============================================
// CURRENT SENSOR VALUES
// =============================================

static float currentWindDir = 0.0f;
static float currentWindSpeed = 0.0f;
static float currentWindGust = 0.0f;

static float ws8xBatteryVoltage = 0.0f;
static float humidityF = 0.0f;
static float temperatureF = 0.0f;

// =============================================
// STATUS
// =============================================

static unsigned long firstReadTime = 0;
static unsigned long lastWindReadTime = 0;

static bool receivedWindDirection = false;
static bool receivedWindSpeed = false;

// =============================================
// HELPER FUNCTIONS
// =============================================

static bool isReadingRecent(
    unsigned long timestamp,
    unsigned long currentTime
)
{
    if (timestamp == 0) {
        return false;
    }

    return (
        currentTime - timestamp <=
        AVG_WINDOW_MS
    );
}

static float valueAfterEquals(const String& line)
{
    int equalsPosition = line.indexOf('=');

    if (equalsPosition < 0) {
        return NAN;
    }

    String value =
        line.substring(equalsPosition + 1);

    value.trim();

    return value.toFloat();
}

static int latestReadingIndex()
{
    if (readingCount <= 0) {
        return -1;
    }

    return (
        writeIndex - 1 + MAX_READINGS
    ) % MAX_READINGS;
}

static void clearReadingArrays()
{
    for (int i = 0; i < MAX_READINGS; i++) {
        windSpeeds[i] = 0.0f;
        windGusts[i] = 0.0f;
        windDirs[i] = 0.0f;
        windTimes[i] = 0;
    }
}

static void storeCalibratedReading(
    float speed,
    float gust,
    float direction,
    unsigned long timestamp
)
{
    if (
        speed < 0.0f ||
        speed >= 100.0f
    ) {
        return;
    }

    if (
        gust < 0.0f ||
        gust >= 100.0f
    ) {
        return;
    }

    if (
        direction < 0.0f ||
        direction >= 360.0f
    ) {
        return;
    }

    windSpeeds[writeIndex] = speed;
    windGusts[writeIndex] = gust;
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

    lastWindReadTime = timestamp;

#if PRINT_COLLECTION == 1
    Serial.printf(
        "[STORED] %d/%d: speed=%.1f m/s, gust=%.1f m/s, direction=%.0f deg\n",
        readingCount,
        MAX_READINGS,
        speed,
        gust,
        direction
    );
#endif
}

static int countRecentReadings()
{
    unsigned long currentTime = millis();
    int validCount = 0;

    for (int i = 0; i < readingCount; i++) {
        if (
            isReadingRecent(
                windTimes[i],
                currentTime
            )
        ) {
            validCount++;
        }
    }

    return validCount;
}

// =============================================
// WIND CALCULATIONS
// =============================================

static float calculateWindSpeedAverage()
{
    unsigned long currentTime = millis();

    float sum = 0.0f;
    int validCount = 0;

    for (int i = 0; i < readingCount; i++) {
        if (
            isReadingRecent(
                windTimes[i],
                currentTime
            )
        ) {
            sum += windSpeeds[i];
            validCount++;
        }
    }

    if (validCount == 0) {
        return currentWindSpeed;
    }

    return sum / validCount;
}

static float calculateWindDirectionAverage()
{
    unsigned long currentTime = millis();

    float sineSum = 0.0f;
    float cosineSum = 0.0f;
    float weightSum = 0.0f;

    int validCount = 0;

    for (int i = 0; i < readingCount; i++) {
        if (
            !isReadingRecent(
                windTimes[i],
                currentTime
            )
        ) {
            continue;
        }

        /*
         * Wind direction has little meaning during complete
         * calm. Only use readings with measurable wind when
         * calculating the circular average.
         */
        if (
            windSpeeds[i] <=
            MIN_WIND_SPEED_FOR_DIR
        ) {
            continue;
        }

        float radians =
            windDirs[i] * PI / 180.0f;

        /*
         * Weight direction by wind speed.
         */
        float weight = windSpeeds[i];

        if (weight < 0.1f) {
            weight = 0.1f;
        }

        sineSum += sin(radians) * weight;
        cosineSum += cos(radians) * weight;
        weightSum += weight;

        validCount++;
    }

    if (
        validCount == 0 ||
        weightSum <= 0.0f
    ) {
        return currentWindDir;
    }

    float averageDirection =
        atan2(
            sineSum / weightSum,
            cosineSum / weightSum
        ) * 180.0f / PI;

    if (averageDirection < 0.0f) {
        averageDirection += 360.0f;
    }

    return averageDirection;
}

static float calculateMaximumGust()
{
    unsigned long currentTime = millis();

    float maximumGust = 0.0f;
    bool foundReading = false;

    for (int i = 0; i < readingCount; i++) {
        if (
            !isReadingRecent(
                windTimes[i],
                currentTime
            )
        ) {
            continue;
        }

        if (
            !foundReading ||
            windGusts[i] > maximumGust
        ) {
            maximumGust = windGusts[i];
            foundReading = true;
        }
    }

    if (!foundReading) {
        return currentWindGust;
    }

    return maximumGust;
}

static float calculateMinimumWind()
{
    unsigned long currentTime = millis();

    float minimumWind = 0.0f;
    bool foundReading = false;

    for (int i = 0; i < readingCount; i++) {
        if (
            !isReadingRecent(
                windTimes[i],
                currentTime
            )
        ) {
            continue;
        }

        if (
            !foundReading ||
            windSpeeds[i] < minimumWind
        ) {
            minimumWind = windSpeeds[i];
            foundReading = true;
        }
    }

    if (!foundReading) {
        return currentWindSpeed;
    }

    return minimumWind;
}

// =============================================
// SERIAL PARSER
// =============================================

static void processSerialLine(String line)
{
    line.trim();

    if (line.length() == 0) {
        return;
    }

#if PRINT_RAW_SERIAL == 1
    Serial.print("[RAW] ");
    Serial.println(line);
#endif

    /*
     * IMPORTANT:
     *
     * Only parse the final calibrated WS80 fields:
     *
     * WindDir   = 255
     * WindSpeed = 0.0
     * WindGust  = 0.5
     *
     * Do not parse the internal diagnostic fields:
     *
     * direction = 304
     * wind = 5
     */

    // -----------------------------------------
    // CALIBRATED WIND DIRECTION
    // -----------------------------------------

    if (line.startsWith("WindDir")) {
        float direction =
            valueAfterEquals(line);

        if (
            !isnan(direction) &&
            direction >= 0.0f &&
            direction < 360.0f
        ) {
            currentWindDir = direction;
            receivedWindDirection = true;

#if PRINT_PARSED_DATA == 1
            Serial.printf(
                "[PARSED] WindDir = %.0f deg\n",
                currentWindDir
            );
#endif
        }

        return;
    }

    // -----------------------------------------
    // CALIBRATED WIND SPEED
    // -----------------------------------------

    if (line.startsWith("WindSpeed")) {
        float speed =
            valueAfterEquals(line);

        if (
            !isnan(speed) &&
            speed >= 0.0f &&
            speed < 100.0f
        ) {
            /*
             * No artificial 3-8 m/s filter.
             * Preserve real calm and strong wind.
             */
            currentWindSpeed = speed;
            receivedWindSpeed = true;

#if PRINT_PARSED_DATA == 1
            Serial.printf(
                "[PARSED] WindSpeed = %.1f m/s\n",
                currentWindSpeed
            );
#endif
        }

        return;
    }

    // -----------------------------------------
    // CALIBRATED WIND GUST
    // -----------------------------------------

    if (line.startsWith("WindGust")) {
        float gust =
            valueAfterEquals(line);

        if (
            !isnan(gust) &&
            gust >= 0.0f &&
            gust < 100.0f
        ) {
            currentWindGust = gust;

#if PRINT_PARSED_DATA == 1
            Serial.printf(
                "[PARSED] WindGust = %.1f m/s\n",
                currentWindGust
            );
#endif

            /*
             * WindGust is the final line in the calibrated
             * wind group. Store one complete reading now.
             */
            if (
                receivedWindDirection &&
                receivedWindSpeed
            ) {
                storeCalibratedReading(
                    currentWindSpeed,
                    currentWindGust,
                    currentWindDir,
                    millis()
                );
            } else {
                Serial.println(
                    "[WS8X] Incomplete calibrated wind block"
                );
            }

            receivedWindDirection = false;
            receivedWindSpeed = false;
        }

        return;
    }

    // -----------------------------------------
    // TEMPERATURE
    // -----------------------------------------

    if (line.startsWith("Temperature")) {
        float temperature =
            valueAfterEquals(line);

        if (
            !isnan(temperature) &&
            temperature > -50.0f &&
            temperature < 100.0f
        ) {
            temperatureF = temperature;

#if PRINT_PARSED_DATA == 1
            Serial.printf(
                "[PARSED] Temperature = %.1f C\n",
                temperatureF
            );
#endif
        }

        return;
    }

    // -----------------------------------------
    // HUMIDITY
    // -----------------------------------------

    if (line.startsWith("Humi")) {
        float humidity =
            valueAfterEquals(line);

        if (
            !isnan(humidity) &&
            humidity >= 0.0f &&
            humidity <= 100.0f
        ) {
            humidityF = humidity;

#if PRINT_PARSED_DATA == 1
            Serial.printf(
                "[PARSED] Humidity = %.1f %%\n",
                humidityF
            );
#endif
        }

        return;
    }

    // -----------------------------------------
    // WS80 BATTERY
    // -----------------------------------------

    if (line.startsWith("BatVoltage")) {
        float voltage =
            valueAfterEquals(line);

        if (
            !isnan(voltage) &&
            voltage > 0.0f &&
            voltage < 10.0f
        ) {
            ws8xBatteryVoltage = voltage;

#if PRINT_PARSED_DATA == 1
            Serial.printf(
                "[PARSED] WS80 battery = %.2f V\n",
                ws8xBatteryVoltage
            );
#endif
        }

        return;
    }
}

// =============================================
// INITIALIZATION
// =============================================

void ws8x_init()
{
    /*
     * Receive-only wiring:
     *
     * WS80 TX  -> CubeCell UART_RX
     * WS80 GND -> CubeCell GND
     * UART_TX  -> physically disconnected
     *
     * UART_TX is still supplied to the CubeCell UART
     * driver because framework 1.6.0 expects both named
     * hardware UART pins.
     */
    Serial1.begin(
        115200,
        SERIAL_8N1,
        UART_RX,
        UART_TX
    );

    Serial1.setTimeout(100);

    clearReadingArrays();

    readingCount = 0;
    writeIndex = 0;

    currentWindDir = 0.0f;
    currentWindSpeed = 0.0f;
    currentWindGust = 0.0f;

    ws8xBatteryVoltage = 0.0f;
    humidityF = 0.0f;
    temperatureF = 0.0f;

    firstReadTime = 0;
    lastWindReadTime = 0;

    receivedWindDirection = false;
    receivedWindSpeed = false;

    Serial.println();
    Serial.println("==========================================");
    Serial.println("[WS8X] INITIALIZED");
    Serial.println("[WS8X] Parsing calibrated WS80 data only");
    Serial.println("[WS8X] WS80 TX -> CubeCell UART_RX");
    Serial.println("[WS8X] UART_TX physically disconnected");
    Serial.println("[WS8X] Baud rate: 115200");

    Serial.printf(
        "[WS8X] Average window: %lu seconds\n",
        AVG_WINDOW_MS / 1000UL
    );

    Serial.println("==========================================");
}

// =============================================
// SERIAL READER
// =============================================

void ws8x_checkSerial()
{
    while (Serial1.available() > 0) {
        String line =
            Serial1.readStringUntil('\n');

        processSerialLine(line);
    }

#if PRINT_COLLECTION == 1
    static unsigned long lastStatusPrint = 0;

    if (
        millis() - lastStatusPrint >=
        10000UL
    ) {
        lastStatusPrint = millis();

        Serial.printf(
            "[COLLECT] %d recent calibrated readings\n",
            countRecentReadings()
        );
    }
#endif
}

// =============================================
// DATA VALIDATION
// =============================================

bool ws8x_has_valid_data()
{
    unsigned long currentTime = millis();

    if (
        readingCount == 0 ||
        lastWindReadTime == 0
    ) {
        static unsigned long lastNoDataPrint = 0;

        if (
            currentTime - lastNoDataPrint >=
            30000UL
        ) {
            lastNoDataPrint = currentTime;

            Serial.println(
                "[COLLECT] No calibrated WS80 data received"
            );
        }

        return false;
    }

    if (
        currentTime - lastWindReadTime >
        DATA_TIMEOUT_MS
    ) {
        Serial.println(
            "[COLLECT] WS80 data stale; payload blocked"
        );

        return false;
    }

    int recentReadings =
        countRecentReadings();

    if (
        recentReadings <
        MIN_CALIBRATED_READINGS
    ) {
        return false;
    }

    /*
     * Wait until a full averaging window has elapsed.
     */
    if (
        firstReadTime != 0 &&
        currentTime - firstReadTime <
        AVG_WINDOW_MS
    ) {
        return false;
    }

    return true;
}

// =============================================
// PUBLIC VALUE GETTERS
// =============================================

float ws8x_get_wind_direction()
{
    return calculateWindDirectionAverage();
}

float ws8x_get_wind_speed()
{
    return calculateWindSpeedAverage();
}

float ws8x_get_wind_gust()
{
    return calculateMaximumGust();
}

float ws8x_get_temperature()
{
    return temperatureF;
}

float ws8x_get_battery_voltage()
{
    return ws8xBatteryVoltage;
}

float ws8x_get_humidity()
{
    return humidityF;
}

bool ws8x_force_zero_mode()
{
    return (
        currentWindSpeed <= 0.0f &&
        currentWindGust <= 0.0f
    );
}

// =============================================
// DEBUG OUTPUT
// =============================================

void ws8x_print_data()
{
    float averageSpeed =
        calculateWindSpeedAverage();

    float averageDirection =
        calculateWindDirectionAverage();

    float maximumGust =
        calculateMaximumGust();

    float minimumWind =
        calculateMinimumWind();

    uint16_t boardBatteryMv =
        getBatteryVoltage();

    Serial.println();
    Serial.println("==========================================");
    Serial.println("WS80 CALIBRATED WEATHER DATA");
    Serial.println("------------------------------------------");

    Serial.printf(
        "Recent readings:  %d\n",
        countRecentReadings()
    );

    Serial.printf(
        "Wind average:     %.1f m/s (%.1f knots)\n",
        averageSpeed,
        averageSpeed * 1.94384f
    );

    Serial.printf(
        "Wind gust:        %.1f m/s (%.1f knots)\n",
        maximumGust,
        maximumGust * 1.94384f
    );

    Serial.printf(
        "Wind minimum:     %.1f m/s (%.1f knots)\n",
        minimumWind,
        minimumWind * 1.94384f
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
        ws8xBatteryVoltage
    );

    Serial.printf(
        "CubeCell battery: %u mV\n",
        boardBatteryMv
    );

    Serial.println("==========================================");
    Serial.println();
}

// =============================================
// LORAWAN PAYLOAD
// =============================================

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

    if (size < LORA_PAYLOAD_SIZE) {
        Serial.printf(
            "[WS8X] ERROR: Payload needs %d bytes; received %d\n",
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
            "[WS8X] Payload blocked: insufficient or stale data"
        );

        return;
    }

    float averageDirection =
        calculateWindDirectionAverage();

    float averageSpeed =
        calculateWindSpeedAverage();

    float maximumGust =
        calculateMaximumGust();

    float minimumWind =
        calculateMinimumWind();

    uint16_t boardBatteryMv =
        getBatteryVoltage();

    int16_t payloadDirection =
        (int16_t)round(averageDirection);

    int16_t payloadAverage =
        (int16_t)round(
            averageSpeed * 10.0f
        );

    int16_t payloadGust =
        (int16_t)round(
            maximumGust * 10.0f
        );

    int16_t payloadMinimum =
        (int16_t)round(
            minimumWind * 10.0f
        );

    int16_t payloadWs8xBattery =
        (int16_t)round(
            ws8xBatteryVoltage * 100.0f
        );

    int16_t payloadHumidity =
        (int16_t)round(
            humidityF * 100.0f
        );

    int16_t payloadTemperature =
        (int16_t)round(
            temperatureF * 10.0f
        );

    uint16_t payloadRain = 0;

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
        &payloadMinimum,
        sizeof(payloadMinimum)
    );
    offset += sizeof(payloadMinimum);

    memcpy(
        &buffer[offset],
        &payloadWs8xBattery,
        sizeof(payloadWs8xBattery)
    );
    offset += sizeof(payloadWs8xBattery);

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
        &boardBatteryMv,
        sizeof(boardBatteryMv)
    );
    offset += sizeof(boardBatteryMv);

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

// =============================================
// RESET AVERAGING WINDOW
// =============================================

void ws8x_reset_counters()
{
    clearReadingArrays();

    readingCount = 0;
    writeIndex = 0;

    firstReadTime = 0;
    lastWindReadTime = 0;

    receivedWindDirection = false;
    receivedWindSpeed = false;

    Serial.println(
        "[WS8X] Averaging window reset"
    );
}