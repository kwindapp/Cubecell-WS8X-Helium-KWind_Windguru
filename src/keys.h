#ifndef SECRETS_H
#define SECRETS_H

// OTAA keys !!!! KEYS ARE MSB !!!!
// Example keys (replace with your actual keys)
// NODE_APP_EUI AKA NODE_JOIN_EUI

#define NODE_DEVICE_EUI {0x60, 0x81, 0xF9, 0xB8, 0x39, 0x5D, 0x8F, 0x97}
#define NODE_APP_EUI    {0x60, 0x81, 0xF9, 0xA1, 0x8A, 0xEB, 0x66, 0xCB} // AKI JOIN_EUI
#define NODE_APP_KEY    {0x19, 0xCA, 0xCF, 0x93, 0x5D, 0xBE, 0x99, 0xD8, 0x29, 0xF7, 0x34, 0x42, 0x05, 0xDC, 0x57, 0x36}

// =============================================
// LORAWAN CONFIGURATION
// =============================================
#define ACTIVE_REGION LORAMAC_REGION_EU868   // EU868 for Europe
#define LORAWAN_CLASS CLASS_A                // Class A
#define LORAWAN_NETMODE true                 // true = OTAA, false = ABP
#define LORAWAN_NET_RESERVE true             // Save network info to flash
#define LORAWAN_UPLINKMODE false             // false = unconfirmed

#endif // SECRETS_H