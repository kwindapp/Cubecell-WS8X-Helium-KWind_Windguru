/*
 * ============================================================================
 *                 KWind - Windguru Network LoRaWAN Firmware
 * ============================================================================
 *
 *  Project:     KWind Live Wind Station Network
 *  Module:      CubeCell AB01 / AB01 V2 LoRaWAN Transmitter
 *  Sensor:      Fine Offset / Ecowitt WS80 and WS85
 *  Integration: KWind and Windguru live-wind data network
 *  Release:     September 2026
 *
 *  Description:
 *  Receives calibrated ultrasonic wind and weather measurements from a
 *  WS80/WS85 sensor over UART, creates an 18-byte LoRaWAN payload, and
 *  transmits the measurements to the KWind live-wind infrastructure for
 *  network integration and forwarding.
 *
 *  Copyright (c) 2026 KWind Hekiumsmartworld KLG Switzerland
 *  All rights reserved.
 *
 *  This source code is proprietary KWind software. Unauthorized copying,
 *  modification, redistribution, publication, sublicensing, or commercial
 *  use of this software, in whole or in part, is prohibited without prior
 *  written permission from the copyright owner.
 *
 *  KWind and Windguru remain the property of their respective owners.
 * ============================================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "ws8x.h"
#include "keys.h"

// Defined by the CubeCell framework
extern bool wakeByUart;

// LoRaWAN library debug output
bool g_lora_debug = true;

// =============================================
// CONFIGURATION
// =============================================

static const uint8_t PAYLOAD_SIZE = 18;
static const uint32_t DATA_RETRY_INTERVAL_MS = 5000UL;

// =============================================
// OTAA PARAMETERS
// =============================================

uint8_t devEui[8] = NODE_DEVICE_EUI;
uint8_t appEui[8] = NODE_APP_EUI;
uint8_t appKey[16] = NODE_APP_KEY;

// =============================================
// ABP PARAMETERS
// =============================================

uint8_t nwkSKey[16] = {
    0x15, 0xB1, 0xD0, 0xEF,
    0xA4, 0x63, 0xDF, 0xBE,
    0x3D, 0x11, 0x18, 0x1E,
    0x1E, 0xC7, 0xDA, 0x85
};

uint8_t appSKey[16] = {
    0xD7, 0x2C, 0x78, 0x75,
    0x8C, 0xDC, 0xCA, 0xBF,
    0x55, 0xEE, 0x4A, 0x77,
    0x8D, 0x16, 0xEF, 0x67
};

uint32_t devAddr = (uint32_t)0x007E6AE1;

// =============================================
// LORAWAN SETTINGS
// =============================================

/*
 * EU868 channels 0-7 enabled.
 *
 * The previous value 0xFF00 enabled bits 8-15 instead
 * of the normal first eight EU868 channels.
 */
uint16_t userChannelsMask[6] = {
    0x00FF,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0000
};

// Region selected in Arduino IDE / PlatformIO
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

// Class selected in Arduino IDE / PlatformIO
DeviceClass_t loraWanClass = LORAWAN_CLASS;

// Normal transmission interval
uint32_t appTxDutyCycle = 60000UL;

// OTAA or ABP
bool overTheAirActivation = LORAWAN_NETMODE;

// Adaptive Data Rate
bool loraWanAdr = false;

// Preserve network session in flash
bool keepNet = LORAWAN_NET_RESERVE;

// Confirmed or unconfirmed uplinks
bool isTxConfirmed = LORAWAN_UPLINKMODE;

// LoRaWAN application port
uint8_t appPort = 2;

// Number of confirmed-uplink attempts
uint8_t confirmedNbTrials = 4;

// =============================================
// STATUS VARIABLES
// =============================================

static unsigned long lastWaitingPrint = 0;
static unsigned long successfulPackets = 0;

// =============================================
// HELPER FUNCTIONS
// =============================================

static void printHexArray(
    const uint8_t* data,
    uint8_t size
)
{
    for (uint8_t i = 0; i < size; i++) {
        Serial.printf("%02X", data[i]);
    }
}

static void printStartupInformation()
{
    Serial.println();
    Serial.println("==========================================");
    Serial.println("CubeCell WS80/WS85 LoRaWAN Transmitter");
    Serial.println("------------------------------------------");

    Serial.print("DevEUI: ");
    printHexArray(devEui, sizeof(devEui));
    Serial.println();

    Serial.print("AppEUI: ");
    printHexArray(appEui, sizeof(appEui));
    Serial.println();

    Serial.printf(
        "Duty cycle: %lu ms (%lu minute(s))\n",
        appTxDutyCycle,
        appTxDutyCycle / 60000UL
    );

    Serial.printf(
        "Payload size: %u bytes\n",
        PAYLOAD_SIZE
    );

    Serial.printf(
        "Region: %d\n",
        loraWanRegion
    );

    Serial.printf(
        "Class: %d\n",
        loraWanClass
    );

    Serial.printf(
        "Activation: %s\n",
        overTheAirActivation ? "OTAA" : "ABP"
    );

    Serial.printf(
        "ADR: %s\n",
        loraWanAdr ? "enabled" : "disabled"
    );

    Serial.printf(
        "Uplink: %s\n",
        isTxConfirmed ? "confirmed" : "unconfirmed"
    );

    Serial.println("==========================================");
    Serial.println();
}

static void printPayload()
{
    Serial.printf(
        "[LORAWAN] Payload size: %u bytes\n",
        appDataSize
    );

    Serial.print("[LORAWAN] Payload HEX: ");

    for (uint8_t i = 0; i < appDataSize; i++) {
        Serial.printf("%02X", appData[i]);
    }

    Serial.println();
}

static void scheduleNextCycle(uint32_t interval)
{
    txDutyCycleTime =
        interval +
        randr(0, APP_TX_DUTYCYCLE_RND);

    Serial.printf(
        "[LORAWAN] Next cycle in approximately %lu seconds\n",
        txDutyCycleTime / 1000UL
    );

    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_SLEEP;
}

// =============================================
// SETUP
// =============================================

void setup()
{
#ifdef INTERVAL_MINUTES
    appTxDutyCycle =
        (uint32_t)INTERVAL_MINUTES *
        60000UL;
#endif

    boardInitMcu();

    Serial.begin(115200);
    delay(3000);

    printStartupInformation();

    // -----------------------------------------
    // Initialize WS80 receive-only UART
    // -----------------------------------------

    Serial.println("[WS8X] Initializing sensor receiver");

    ws8x_init();

    Serial.println("[WS8X] Sensor receiver initialized");
    Serial.println("[WS8X] WS80 TX -> CubeCell UART_RX");
    Serial.println("[WS8X] CubeCell UART_TX disconnected");

    // -----------------------------------------
    // Configure LoRaWAN
    // -----------------------------------------

    appDataSize = PAYLOAD_SIZE;

    /*
     * Allow UART activity to wake the CubeCell so
     * incoming WS80 data can continue to be processed.
     */
    wakeByUart = true;

    /*
     * Start the standard CubeCell LoRaWAN state machine.
     */
    deviceState = DEVICE_STATE_INIT;

    /*
     * Uses the framework's network-reserve setting.
     * When a valid stored session exists, joining can
     * be skipped according to the framework settings.
     */
    LoRaWAN.ifskipjoin();

    Serial.println();
    Serial.println("[SYSTEM] Ready");
    Serial.println("[SYSTEM] Collecting WS80 readings");
    Serial.println("[SYSTEM] Initializing LoRaWAN");
    Serial.println();
}

// =============================================
// MAIN LOOP
// =============================================

void loop()
{
    /*
     * Always process UART data before handling the
     * LoRaWAN state machine.
     */
    ws8x_checkSerial();

    switch (deviceState)
    {
        // =====================================
        // INITIALIZE LORAWAN
        // =====================================

        case DEVICE_STATE_INIT:
        {
            Serial.println(
                "[LORAWAN] Initializing stack"
            );

            LoRaWAN.init(
                loraWanClass,
                loraWanRegion
            );

            deviceState = DEVICE_STATE_JOIN;

            Serial.println(
                "[LORAWAN] Stack initialized"
            );

            break;
        }

        // =====================================
        // JOIN NETWORK
        // =====================================

        case DEVICE_STATE_JOIN:
        {
            /*
             * Important:
             *
             * Do not manually overwrite deviceState here.
             * LoRaWAN.join() changes it to DEVICE_STATE_SEND
             * after a successful join.
             */
            Serial.println(
                "[LORAWAN] Joining network"
            );

            LoRaWAN.join();

            break;
        }

        // =====================================
        // PREPARE AND SEND PAYLOAD
        // =====================================

        case DEVICE_STATE_SEND:
        {
            /*
             * Only upload after MIN_READINGS valid and
             * recent WS80 readings have been collected.
             */
            if (!ws8x_has_valid_data()) {
                if (
                    millis() - lastWaitingPrint >=
                    10000UL
                ) {
                    lastWaitingPrint = millis();

                    Serial.println(
                        "[LORAWAN] Waiting for WS80 readings"
                    );

                    ws8x_print_data();
                }

                /*
                 * Retry soon instead of waiting for the
                 * complete normal transmission interval.
                 */
                scheduleNextCycle(
                    DATA_RETRY_INTERVAL_MS
                );

                break;
            }

            Serial.println();
            Serial.println(
                "[LORAWAN] Preparing uplink"
            );

            appDataSize = PAYLOAD_SIZE;

            /*
             * Clear the complete LoRaWAN payload before
             * writing the new sensor values.
             */
            memset(
                appData,
                0,
                appDataSize
            );

            ws8x_populate_lora_buffer(
                appData,
                appDataSize
            );

            printPayload();

            /*
             * LoRaWAN.send() uses appData, appDataSize
             * and appPort from the global variables.
             */
            LoRaWAN.send();

            successfulPackets++;

            Serial.printf(
                "[LORAWAN] Uplink submitted, packet #%lu\n",
                successfulPackets
            );

            /*
             * Start a fresh averaging interval only after
             * the packet has been submitted.
             */
            ws8x_reset_counters();

            deviceState = DEVICE_STATE_CYCLE;

            break;
        }

        // =====================================
        // NORMAL TRANSMISSION CYCLE
        // =====================================

        case DEVICE_STATE_CYCLE:
        {
            scheduleNextCycle(
                appTxDutyCycle
            );

            break;
        }

        // =====================================
        // LOW-POWER SLEEP
        // =====================================

        case DEVICE_STATE_SLEEP:
        {
            /*
             * Read pending WS80 UART bytes before entering
             * the CubeCell low-power handler.
             */
            ws8x_checkSerial();

            LoRaWAN.sleep();

            break;
        }

        // =====================================
        // RECOVERY
        // =====================================

        default:
        {
            Serial.println(
                "[SYSTEM] Invalid state; restarting LoRaWAN"
            );

            deviceState = DEVICE_STATE_INIT;

            break;
        }
    }

    delay(10);
}

// =============================================
// DOWNLINK HANDLER
// =============================================

void downLinkDataHandle(
    McpsIndication_t* mcpsIndication
)
{
    if (mcpsIndication == NULL) {
        Serial.println(
            "[DOWNLINK] Invalid indication"
        );

        return;
    }

    Serial.println();
    Serial.println("==========================================");
    Serial.println("LORAWAN DOWNLINK RECEIVED");
    Serial.println("------------------------------------------");

    Serial.printf(
        "Port: %u\n",
        mcpsIndication->Port
    );

    Serial.printf(
        "Size: %u bytes\n",
        mcpsIndication->BufferSize
    );

    Serial.print("HEX: ");

    for (
        uint8_t i = 0;
        i < mcpsIndication->BufferSize;
        i++
    ) {
        Serial.printf(
            "%02X ",
            mcpsIndication->Buffer[i]
        );
    }

    Serial.println();

    Serial.print("ASCII: ");

    for (
        uint8_t i = 0;
        i < mcpsIndication->BufferSize;
        i++
    ) {
        uint8_t character =
            mcpsIndication->Buffer[i];

        if (
            character >= 32 &&
            character <= 126
        ) {
            Serial.print((char)character);
        } else {
            Serial.print('.');
        }
    }

    Serial.println();
    Serial.println("==========================================");

    // -----------------------------------------
    // REBOOT COMMAND
    // -----------------------------------------

    if (mcpsIndication->BufferSize == 6) {
        char command[7];

        memcpy(
            command,
            mcpsIndication->Buffer,
            6
        );

        command[6] = '\0';

        if (strcmp(command, "reboot") == 0) {
            Serial.println(
                "[DOWNLINK] Reboot command received"
            );

            delay(1000);
            NVIC_SystemReset();
            return;
        }
    }

    // -----------------------------------------
    // DUTY-CYCLE COMMAND
    // -----------------------------------------

    if (mcpsIndication->BufferSize == 1) {
        uint8_t value =
            mcpsIndication->Buffer[0];

        /*
         * ASCII values "1" through "9" set the normal
         * transmission interval in minutes.
         */
        if (value >= '1' && value <= '9') {
            uint8_t minutes =
                value - '0';

            appTxDutyCycle =
                (uint32_t)minutes *
                60000UL;

            Serial.printf(
                "[DOWNLINK] Duty cycle changed to %u minute(s), %lu ms\n",
                minutes,
                appTxDutyCycle
            );
        }
    }
}