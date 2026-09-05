#ifndef SECRETS_H
#define SECRETS_H

// OTAA keys !!!! KEYS ARE MSB !!!!
// Example keys (replace with your actual keys)
// NODE_APP_EUI AKA NODE_JOIN_EUI

#define NODE_DEVICE_EUI {0x60, 0x81, 0xF9, 0xC7, 0xC3, 0xF3, 0xCC, 0xF5}
#define NODE_APP_EUI    {0x60, 0x81, 0xF9, 0xFF, 0x01, 0x27, 0xCE, 0x20} // AKI JOIN_EUI
#define NODE_APP_KEY    {0xBA, 0x86, 0x05, 0x11, 0x6F, 0x4D, 0x7F, 0x7F, 0xCA, 0xB3, 0x9C, 0x49, 0x99, 0xD8, 0x28, 0xC1}

// =============================================
// LORAWAN CONFIGURATION
// =============================================
#define ACTIVE_REGION LORAMAC_REGION_EU868   // EU868 for Europe
#define LORAWAN_CLASS CLASS_A                // Class A
#define LORAWAN_NETMODE true                 // true = OTAA, false = ABP
#define LORAWAN_NET_RESERVE true             // Save network info to flash
#define LORAWAN_UPLINKMODE false             // false = unconfirmed

#endif // SECRETS_H