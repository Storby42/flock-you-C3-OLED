#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <NimBLEAdvertisedDevice.h>
#include <ArduinoJson.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <EEPROM.h>
#include "esp_wifi.h"
#include "esp_wifi_types.h"

//display thingy
#include <U8g2lib.h>
#include <Wire.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define WIPE_EEPROM_ON_STARTUP false// Set to true to clear saved device suffixes on startup
// Hardware Configuration
#define BUZZER_PIN 4  // GPIO3 (D2) - PWM capable pin on Xiao ESP32 S3
#define LED_PIN 8

// Audio Configuration
#define LOW_FREQ 300      // Boot sequence - low pitch
#define HIGH_FREQ 400     // Boot sequence - high pitch & detection alert
#define DETECT_FREQ 800  // Detection alert - high pitch (faster beeps)
#define HEARTBEAT_FREQ 600 // Heartbeat pulse frequency
#define BOOT_BEEP_DURATION 300   // Boot beep duration
#define DETECT_BEEP_DURATION 50 // Detection beep duration (faster)
#define HEARTBEAT_DURATION 15   // Short heartbeat pulse

// WiFi Promiscuous Mode Configuration
#define MAX_CHANNEL 13
#define CHANNEL_HOP_INTERVAL 500  // milliseconds

// BLE SCANNING CONFIGURATION
#define BLE_SCAN_DURATION 1    // Seconds
#define BLE_SCAN_INTERVAL 5000 // Milliseconds between scans
static unsigned long last_ble_scan = 0;

// Detection Pattern Limits
#define MAX_SSID_PATTERNS 10
#define MAX_MAC_PATTERNS 1024
#define MAX_DEVICE_NAMES 20

//oled thingy continued

#ifndef BIRD_BITMAP_H
#define BIRD_BITMAP_H

int new_device_count = 0;
int detected_device_count = 0;
int saved_device_count = 0;
char detected_devices[MAX_MAC_PATTERNS][32];
static uint8_t saved_devices[MAX_MAC_PATTERNS][6];; // Store up to max_mac_patterns MAC addresses

String current_action = "booting";

// Width: 70
// Height: 40
static const unsigned char PROGMEM bird_bitmap[360] = {
  0xFF,0xFD,0xEF,0xFF,0xFF,0xF7,0xDF,0xFD,0x3F,
  0xD1,0x71,0x94,0xFF,0xFF,0xE3,0xDF,0xE6,0x36,
  0xDC,0x2D,0xC7,0xFF,0x80,0xE1,0x5F,0xDA,0x36,
  0xDD,0xAC,0xE5,0x0E,0x80,0xE0,0x3F,0x99,0x36,
  0xD1,0xAC,0xC7,0xBF,0x1D,0xE0,0x3F,0x99,0x36,
  0xDD,0x25,0xD5,0x03,0x1E,0xE0,0x1F,0xDB,0x36,
  0x1D,0x73,0x96,0x00,0x08,0xA0,0x3F,0xE7,0x38,
  0xFB,0xFF,0x3E,0x00,0x00,0x20,0xFF,0xFF,0x3F,
  0xBF,0xFF,0x1F,0x00,0x00,0x00,0xFE,0xFF,0x3F,
  0xFF,0xFF,0x0F,0x00,0x00,0x60,0xDC,0xFF,0x3F,
  0xFF,0xFF,0x07,0x00,0x00,0xE0,0xEC,0xFF,0x3F,
  0xDF,0xFE,0x03,0x00,0x00,0x80,0xF9,0xFF,0x1F,
  0xFF,0xFF,0x01,0x00,0x00,0x00,0xD8,0xFF,0x3F,
  0xFF,0xFF,0x00,0x00,0x00,0x00,0xF0,0xFF,0x3F,
  0xFF,0x3F,0x00,0x00,0x00,0x00,0xF8,0xDF,0x3F,
  0xFF,0x0F,0x00,0x00,0x00,0x00,0xF0,0xFF,0x3F,
  0xFF,0x1F,0x00,0x00,0x00,0x00,0xF0,0xFF,0x3F,
  0xFF,0x0F,0x00,0x00,0x00,0x00,0xF0,0xFF,0x37,
  0xFF,0x07,0x00,0x00,0x00,0x00,0xE0,0xFF,0x3F,
  0xFF,0x03,0xC0,0x01,0x00,0x00,0xE0,0xFF,0x3F,
  0xFF,0x07,0xE0,0x0F,0x00,0x00,0xE0,0xFF,0x3E,
  0xFF,0x07,0xF0,0x3F,0x00,0x00,0xE0,0xFF,0x3F,
  0xFF,0x17,0xFE,0x1F,0x00,0x00,0xE0,0xFB,0x3F,
  0xFF,0x0F,0xE7,0x0F,0x00,0x00,0x60,0xFF,0x3F,
  0xFF,0xFF,0xC7,0x07,0x00,0x00,0xE0,0xFF,0x3F,
  0xFF,0xEF,0xE7,0x01,0x60,0x00,0xE0,0xFF,0x3F,
  0xFF,0xFF,0xC6,0x04,0xE0,0x03,0xE0,0xFF,0x3F,
  0xFF,0xFF,0xCD,0x00,0xE0,0x07,0xE0,0xFD,0x3D,
  0xFF,0xFF,0x9F,0x03,0xE0,0x0F,0xE0,0xFF,0x3F,
  0xFF,0xF7,0x1F,0x01,0xE0,0x0F,0xE0,0xFE,0x37,
  0xFF,0xFF,0x3F,0x07,0xE0,0x07,0xE0,0xFF,0x3F,
  0xBE,0xFF,0x7F,0x00,0xE0,0x0F,0xE0,0xEF,0x27,
  0xFF,0xFF,0xFF,0x10,0xE5,0x07,0xE0,0xFF,0x3F,
  0xFF,0xFF,0xFF,0x80,0xFF,0x03,0xE0,0xFF,0x3F,
  0xFF,0xFF,0xFF,0x43,0x7F,0x10,0xE8,0xFF,0x3F,
  0xFF,0xFF,0xBF,0x0F,0x00,0x08,0xF8,0xFF,0x3F,
  0xFF,0xFF,0xFF,0x5F,0x00,0x1F,0xF0,0xFF,0x3F,
  0xFF,0xFF,0xDF,0xFF,0xFF,0x0F,0xF0,0xFF,0x3F,
  0xFF,0xFF,0xFF,0xFB,0xFF,0x3F,0xF2,0xF7,0x3F,
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFF,0x3F
};
#endif // BIRD_BITMAP_H

#ifndef FLY00_BITMAP_H
#define FLY00_BITMAP_H

// Width: 32
// Height: 24

static const unsigned char PROGMEM fly00_bitmap[96] = {
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x1E,0x00,
  0x00,0xC0,0x2B,0x00,
  0x00,0x7F,0x5F,0x00,
  0x80,0x0A,0x80,0x00,
  0x00,0x5F,0x1D,0x00,
  0x00,0x08,0x18,0x00,
  0x00,0x50,0x0F,0x00,
  0x00,0x08,0x0E,0x00,
  0x00,0x58,0x0F,0x00,
  0x00,0x84,0x0A,0x00,
  0x00,0xD4,0x07,0x00,
  0x00,0x24,0x03,0x00,
  0x00,0x18,0x00,0x00
};

#endif // FLY00_BITMAP_H

#ifndef FLY01_BITMAP_H
#define FLY01_BITMAP_H

// Width: 32
// Height: 24

static const unsigned char PROGMEM fly01_bitmap[96] = {
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0xB0,0x1E,0x00,
  0x00,0x08,0x2B,0x00,
  0x00,0x57,0x5F,0x00,
  0x80,0x02,0x82,0x00,
  0x00,0x57,0x7D,0x00,
  0x00,0x01,0x1A,0x00,
  0x00,0xF5,0x0F,0x00,
  0x00,0xD1,0x06,0x00,
  0x00,0x4D,0x03,0x00,
  0x00,0x86,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00
};

#endif // FLY01_BITMAP_H

#ifndef FLY02_BITMAP_H
#define FLY02_BITMAP_H

// Width: 32
// Height: 24

static const unsigned char PROGMEM fly02_bitmap[96] = {
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x78,0x1C,0x00,
  0x00,0xD2,0x1C,0x00,
  0x00,0x01,0x2B,0x00,
  0x00,0x55,0x1E,0x00,
  0x00,0x11,0x3E,0x00,
  0x00,0x4E,0x3F,0x00,
  0x00,0x1B,0x0A,0x00,
  0x80,0x3F,0xD7,0x00,
  0x00,0x3F,0x78,0x00,
  0x00,0xE0,0x17,0x00,
  0x00,0xC0,0x0A,0x00,
  0x00,0x40,0x07,0x00,
  0x00,0xC0,0x03,0x00,
  0x00,0x80,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00
};

#endif // FLY02_BITMAP_H
#ifndef FLY03_BITMAP_H
#define FLY03_BITMAP_H

// Width: 32
// Height: 24

static const unsigned char PROGMEM fly03_bitmap[96] = {
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x70,0x06,0x00,
  0x00,0x08,0x0B,0x00,
  0x00,0xD8,0x1F,0x00,
  0x00,0x88,0x1A,0x00,
  0x00,0x58,0x1F,0x00,
  0x00,0x10,0x1B,0x00,
  0x00,0x50,0x1B,0x00,
  0x00,0x10,0x2A,0x00,
  0x00,0x5B,0x5F,0x00,
  0x80,0x2A,0x82,0x00,
  0x00,0xFF,0x7F,0x00,
  0x00,0xA0,0x1A,0x00,
  0x00,0xC0,0x0F,0x00,
  0x00,0xC0,0x06,0x00,
  0x00,0x40,0x03,0x00,
  0x00,0x80,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00
};

#endif // FLY03_BITMAP_H

int animframe = 0;

// there is no 72x40 constructor in u8g2 hence the 72x40 screen is
// mapped in the middle of the 132x64 pixel buffer of the SSD1306 controller
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 6, 5);
int width = 70;
int height = 40;
int xOffset = 30; // = (132-w)/2
int yOffset = 24; // = (64-h)/2

// ============================================================================
// DETECTION PATTERNS (Extracted from Real Flock Safety Device Databases)
// ============================================================================

// WiFi SSID patterns to detect (case-insensitive)
static const char* wifi_ssid_patterns[] = {
    "flock",        // Standard Flock Safety naming
    "Flock",        // Capitalized variant
    "FLOCK",        // All caps variant
    "FS Ext Battery", // Flock Safety Extended Battery devices
    "Penguin",      // Penguin surveillance devices
    "Pigvision"    // Pigvision surveillance systems
};

// Known Flock Safety MAC address prefixes (from real device databases)
static const char* mac_prefixes[] = {
    // FS Ext Battery devices
    "58:8e:81", "cc:cc:cc", "ec:1b:bd", "90:35:ea", "04:0d:84", 
    "f0:82:c0", "1c:34:f1", "38:5b:44", "94:34:69", "b4:e3:f9",
    
    // Flock WiFi devices
    "70:c9:4e", "3c:91:80", "d8:f3:bc", "80:30:49", "14:5a:fc",
    "74:4c:a1", "08:3a:88", "9c:2f:9d", "94:08:53", "e4:aa:ea"
    
    // Penguin devices - these are NOT OUI based, so use local ouis
    // from the wigle.net db relative to your location 
    // "cc:09:24", "ed:c7:63", "e8:ce:56", "ea:0c:ea", "d8:8f:14",
    // "f9:d9:c0", "f1:32:f9", "f6:a0:76", "e4:1c:9e", "e7:f2:43",
    // "e2:71:33", "da:91:a9", "e1:0e:15", "c8:ae:87", "f4:ed:b2",
    // "d8:bf:b5", "ee:8f:3c", "d7:2b:21", "ea:5a:98"
};

// Device name patterns for BLE advertisement detection
static const char* device_name_patterns[] = {
    "FS Ext Battery",  // Flock Safety Extended Battery
    "Penguin",         // Penguin surveillance devices
    "Flock",           // Standard Flock Safety devices
    "Pigvision"        // Pigvision surveillance systems
};

// ============================================================================
// RAVEN SURVEILLANCE DEVICE UUID PATTERNS
// ============================================================================
// These UUIDs are specific to Raven surveillance devices (acoustic gunshot detection)
// Source: raven_configurations.json - firmware versions 1.1.7, 1.2.0, 1.3.1

// Raven Device Information Service (used across all firmware versions)
#define RAVEN_DEVICE_INFO_SERVICE       "0000180a-0000-1000-8000-00805f9b34fb"

// Raven GPS Location Service (firmware 1.2.0+)
#define RAVEN_GPS_SERVICE               "00003100-0000-1000-8000-00805f9b34fb"

// Raven Power/Battery Service (firmware 1.2.0+)
#define RAVEN_POWER_SERVICE             "00003200-0000-1000-8000-00805f9b34fb"

// Raven Network Status Service (firmware 1.2.0+)
#define RAVEN_NETWORK_SERVICE           "00003300-0000-1000-8000-00805f9b34fb"

// Raven Upload Statistics Service (firmware 1.2.0+)
#define RAVEN_UPLOAD_SERVICE            "00003400-0000-1000-8000-00805f9b34fb"

// Raven Error/Failure Service (firmware 1.2.0+)
#define RAVEN_ERROR_SERVICE             "00003500-0000-1000-8000-00805f9b34fb"

// Health Thermometer Service (firmware 1.1.7)
#define RAVEN_OLD_HEALTH_SERVICE        "00001809-0000-1000-8000-00805f9b34fb"

// Location and Navigation Service (firmware 1.1.7)
#define RAVEN_OLD_LOCATION_SERVICE      "00001819-0000-1000-8000-00805f9b34fb"

// Known Raven service UUIDs for detection
static const char* raven_service_uuids[] = {
    RAVEN_DEVICE_INFO_SERVICE,    // Device info (all versions)
    RAVEN_GPS_SERVICE,            // GPS data (1.2.0+)
    RAVEN_POWER_SERVICE,          // Battery/Solar (1.2.0+)
    RAVEN_NETWORK_SERVICE,        // LTE/WiFi status (1.2.0+)
    RAVEN_UPLOAD_SERVICE,         // Upload stats (1.2.0+)
    RAVEN_ERROR_SERVICE,          // Error tracking (1.2.0+)
    RAVEN_OLD_HEALTH_SERVICE,     // Old health service (1.1.7)
    RAVEN_OLD_LOCATION_SERVICE    // Old location service (1.1.7)
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

static uint8_t current_channel = 1;
static unsigned long last_channel_hop = 0;
static bool triggered = false;
static bool device_in_range = false;
static unsigned long last_detection_time = 0;
static unsigned long last_heartbeat = 0;
static NimBLEScan* pBLEScan;



// ============================================================================
// AUDIO SYSTEM
// ============================================================================

void beep(int frequency, int duration_ms)
{
    tone(BUZZER_PIN, frequency, duration_ms);
    delay(duration_ms + 50);
}

void boot_beep_sequence()
{
    printf("Initializing audio system...\n");
    printf("Playing boot sequence: Low -> High pitch\n");
    beep(LOW_FREQ, BOOT_BEEP_DURATION);
    beep(HIGH_FREQ, BOOT_BEEP_DURATION);
    printf("Audio system ready\n\n");
}

void flock_detected_beep_sequence(int beeps)
{
    printf("FLOCK SAFETY DEVICE DETECTED!\n");
    printf("Playing alert sequence: 3 fast high-pitch beeps\n");
    for (int i = 0; i < beeps; i++) {
        beep(DETECT_FREQ, DETECT_BEEP_DURATION);
        if (i < beeps - 1) delay(50); // Short gap between beeps
    }
    printf("Detection complete - device identified!\n\n");
    
    // Mark device as in range and start heartbeat tracking
    device_in_range = true;
    last_detection_time = millis();
    last_heartbeat = millis();
}

void heartbeat_pulse()
{
    printf("Heartbeat: Device still in range\n");
    beep(HEARTBEAT_FREQ, HEARTBEAT_DURATION);
    delay(5);
    beep(HEARTBEAT_FREQ, HEARTBEAT_DURATION);
}

// ============================================================================
// Device List Handler
// ============================================================================

int append_mac_to_device_list(const uint8_t* mac)
{
    uint8_t mac_suffix[3];
    memcpy(mac_suffix, &mac[3], 3);
    
    // Check if MAC is already in the list
    for (size_t i = 0; i < detected_device_count; i++) {
        if (memcmp(detected_devices[i], mac, 6) == 0) {
            printf("MAC already in detected_devices list\n");
            return 2; // Already in list
        }
    }
    
    // Add MAC to the list if there's space
    if (detected_device_count < MAX_MAC_PATTERNS) {
        memcpy(detected_devices[detected_device_count], mac, 6);
        detected_device_count++;
        bool suffix_found = false;
        for (int i = 0; i < saved_device_count; i++) {
            if (memcmp(&saved_devices[i][3], mac_suffix, 3) == 0) {
                suffix_found = true;
                break;
            }
        }

        if (!suffix_found) {
            if (saved_device_count < MAX_MAC_PATTERNS) {
                memcpy(saved_devices[saved_device_count], mac, 6);
                saved_device_count++;
                printf("Saved device suffix added\n");
                {
                    int index = saved_device_count - 1; // index of the entry we just added
                    int base = 2 + index * 3; // EEPROM layout: bytes 0-1 = count, then 3 bytes per suffix

                    // Write the 3-byte MAC suffix (bytes 3..5 of full MAC) to EEPROM
                    EEPROM.write(base + 0, mac[3]);
                    EEPROM.write(base + 1, mac[4]);
                    EEPROM.write(base + 2, mac[5]);

                    // Update saved_device_count in EEPROM (little-endian uint16 at addr 0..1)
                    EEPROM.write(0, saved_device_count & 0xFF);
                    EEPROM.write(1, (saved_device_count >> 8) & 0xFF);

                    // Commit changes to flash
                    if (EEPROM.commit()) {
                        Serial.printf("EEPROM: stored suffix #%d at addr %d -> %02x:%02x:%02x\n",
                                      index, base, mac[3], mac[4], mac[5]);
                    } else {
                        Serial.println("EEPROM commit failed!");
                    }
                }
            } else {
                printf("Saved devices array full, cannot store suffix\n");
            }
            new_device_count++;
            return 8;
        } 
        else {
            printf("MAC suffix already present in saved_devices\n");
            return 4;
        }
    }
    else {
        printf("Device list full, cannot add new MAC\n");
        return 4;
    }
}

// ============================================================================
// JSON OUTPUT FUNCTIONS
// ============================================================================

void output_wifi_detection_json(const char* ssid, const uint8_t* mac, int rssi, const char* detection_type)
{
    DynamicJsonDocument doc(2048);
    
    // Core detection info
    doc["timestamp"] = millis();
    doc["detection_time"] = String(millis() / 1000.0, 3) + "s";
    doc["protocol"] = "wifi";
    doc["detection_method"] = detection_type;
    doc["alert_level"] = "HIGH";
    doc["device_category"] = "FLOCK_SAFETY";
    
    // WiFi specific info
    doc["ssid"] = ssid;
    doc["ssid_length"] = strlen(ssid);
    doc["rssi"] = rssi;
    doc["signal_strength"] = rssi > -50 ? "STRONG" : (rssi > -70 ? "MEDIUM" : "WEAK");
    doc["channel"] = current_channel;
    
    // MAC address info
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    doc["mac_address"] = mac_str;
    
    char mac_prefix[9];
    snprintf(mac_prefix, sizeof(mac_prefix), "%02x:%02x:%02x", mac[0], mac[1], mac[2]);
    doc["mac_prefix"] = mac_prefix;
    doc["vendor_oui"] = mac_prefix;
    
    // Detection pattern matching
    bool ssid_match = false;
    bool mac_match = false;
    
    for (int i = 0; i < sizeof(wifi_ssid_patterns)/sizeof(wifi_ssid_patterns[0]); i++) {
        if (strcasestr(ssid, wifi_ssid_patterns[i])) {
            doc["matched_ssid_pattern"] = wifi_ssid_patterns[i];
            doc["ssid_match_confidence"] = "HIGH";
            ssid_match = true;
            break;
        }
    }
    
    for (int i = 0; i < sizeof(mac_prefixes)/sizeof(mac_prefixes[0]); i++) {
        if (strncasecmp(mac_prefix, mac_prefixes[i], 8) == 0) {
            doc["matched_mac_pattern"] = mac_prefixes[i];
            doc["mac_match_confidence"] = "HIGH";
            mac_match = true;
            break;
        }
    }
    
    // Detection summary
    doc["detection_criteria"] = ssid_match && mac_match ? "SSID_AND_MAC" : (ssid_match ? "SSID_ONLY" : "MAC_ONLY");
    doc["threat_score"] = ssid_match && mac_match ? 100 : (ssid_match || mac_match ? 85 : 70);
    
    // Frame type details
    if (strcmp(detection_type, "probe_request") == 0 || strcmp(detection_type, "probe_request_mac") == 0) {
        doc["frame_type"] = "PROBE_REQUEST";
        doc["frame_description"] = "Device actively scanning for networks";
    } else {
        doc["frame_type"] = "BEACON";
        doc["frame_description"] = "Device advertising its network";
    }
    
    String json_output;
    serializeJson(doc, json_output);
    Serial.println(json_output);
    //the oled finale
    u8g2.clearBuffer(); // clear the internal memory
    //u8g2.drawFrame(xOffset, yOffset, width, height); //draw a frame around the border
    u8g2.setFont(u8g2_font_tiny5_tf);
    u8g2.setCursor(xOffset, yOffset+5);
    u8g2.printf("SSID: %s", doc["ssid"].as<String>().c_str());
    u8g2.setCursor(xOffset, yOffset+11);
    u8g2.printf("%s", doc["mac_address"].as<String>().c_str());
    u8g2.setFont(u8g2_font_u8glib_4_tf);
    u8g2.setCursor(xOffset, yOffset+16);
    u8g2.printf("Type: %s, %s,", doc["protocol"].as<String>().c_str(), doc["detection_method"].as<String>().c_str());
    u8g2.setCursor(xOffset, yOffset+22);
    u8g2.printf("%s", doc["detection_criteria"].as<String>().c_str());
    u8g2.setCursor(xOffset, yOffset+28);
    u8g2.printf("Threat: %s", doc["threat_score"].as<String>().c_str());
    u8g2.setCursor(xOffset, yOffset+34);
    u8g2.printf("RSSI: %s, %s", doc["rssi"].as<String>().c_str(), doc["signal_strength"].as<String>().c_str());
    u8g2.sendBuffer(); // transfer internal memory to the display

    digitalWrite(LED_PIN, LOW);
    delay(5);
    digitalWrite(LED_PIN, HIGH);
}

void output_ble_detection_json(const char* mac, const char* name, int rssi, const char* detection_method)
{
    DynamicJsonDocument doc(2048);
    
    // Core detection info
    doc["timestamp"] = millis();
    doc["detection_time"] = String(millis() / 1000.0, 3) + "s";
    doc["protocol"] = "BLE";
    doc["detection_method"] = detection_method;
    doc["alert_level"] = "HIGH";
    doc["device_category"] = "FLOCK_SAFETY";
    
    // BLE specific info
    doc["mac_address"] = mac;
    doc["rssi"] = rssi;
    doc["signal_strength"] = rssi > -50 ? "STRONG" : (rssi > -70 ? "MEDIUM" : "WEAK");
    
    // Device name info
    if (name && strlen(name) > 0) {
        doc["device_name"] = name;
        doc["device_name_length"] = strlen(name);
        doc["has_device_name"] = true;
    } else {
        doc["device_name"] = "";
        doc["device_name_length"] = 0;
        doc["has_device_name"] = false;
    }
    
    // MAC address analysis
    char mac_prefix[9];
    strncpy(mac_prefix, mac, 8);
    mac_prefix[8] = '\0';
    doc["mac_prefix"] = mac_prefix;
    doc["vendor_oui"] = mac_prefix;
    
    // Detection pattern matching
    bool name_match = false;
    bool mac_match = false;
    
    // Check MAC prefix patterns
    for (int i = 0; i < sizeof(mac_prefixes)/sizeof(mac_prefixes[0]); i++) {
        if (strncasecmp(mac, mac_prefixes[i], strlen(mac_prefixes[i])) == 0) {
            doc["matched_mac_pattern"] = mac_prefixes[i];
            doc["mac_match_confidence"] = "HIGH";
            mac_match = true;
            break;
        }
    }
    
    // Check device name patterns
    if (name && strlen(name) > 0) {
        for (int i = 0; i < sizeof(device_name_patterns)/sizeof(device_name_patterns[0]); i++) {
            if (strcasestr(name, device_name_patterns[i])) {
                doc["matched_name_pattern"] = device_name_patterns[i];
                doc["name_match_confidence"] = "HIGH";
                name_match = true;
                break;
            }
        }
    }
    
    // Detection summary
    doc["detection_criteria"] = name_match && mac_match ? "NAME_AND_MAC" : 
                               (name_match ? "NAME_ONLY" : "MAC_ONLY");
    doc["threat_score"] = name_match && mac_match ? 100 : 
                         (name_match || mac_match ? 85 : 70);
    
    // BLE advertisement type analysis
    doc["advertisement_type"] = "BLE_ADVERTISEMENT";
    doc["advertisement_description"] = "Bluetooth Low Energy device advertisement";
    
    // Detection method details
    if (strcmp(detection_method, "mac_prefix") == 0) {
        doc["primary_indicator"] = "MAC_ADDRESS";
        doc["detection_reason"] = "MAC address matches known Flock Safety prefix";
    } else if (strcmp(detection_method, "device_name") == 0) {
        doc["primary_indicator"] = "DEVICE_NAME";
        doc["detection_reason"] = "Device name matches Flock Safety pattern";
    }
    
    String json_output;
    serializeJson(doc, json_output);
    Serial.println(json_output);

    u8g2.clearBuffer(); // clear the internal memory
    //u8g2.drawFrame(xOffset, yOffset, width, height); //draw a frame around the border
    u8g2.setFont(u8g2_font_tiny5_tf);
    u8g2.setCursor(xOffset, yOffset+5);
    u8g2.printf("Name: %s", doc["device_name"].as<String>().c_str());
    u8g2.setCursor(xOffset, yOffset+11);
    u8g2.printf("%s", doc["mac_address"].as<String>().c_str());
    u8g2.setFont(u8g2_font_u8glib_4_tf);
    u8g2.setCursor(xOffset, yOffset+16);
    u8g2.printf("Type: %s, %s,", doc["protocol"].as<String>().c_str(), doc["detection_method"].as<String>().c_str());
    u8g2.setCursor(xOffset, yOffset+22);
    u8g2.printf("%s", doc["detection_criteria"].as<String>().c_str());
    u8g2.setCursor(xOffset, yOffset+28);
    u8g2.printf("Threat: %s", doc["threat_score"].as<String>().c_str());
    u8g2.setCursor(xOffset, yOffset+34);
    u8g2.printf("RSSI: %s, %s", doc["rssi"].as<String>().c_str(), doc["signal_strength"].as<String>().c_str());
    u8g2.sendBuffer(); // transfer internal memory to the display

    digitalWrite(LED_PIN, LOW);
    delay(5);
    digitalWrite(LED_PIN, HIGH);
    
}

// ============================================================================
// DETECTION HELPER FUNCTIONS
// ============================================================================

bool check_mac_prefix(const uint8_t* mac)
{
    char mac_str[9];  // Only need first 3 octets for prefix check
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x", mac[0], mac[1], mac[2]);
    
    for (int i = 0; i < sizeof(mac_prefixes)/sizeof(mac_prefixes[0]); i++) {
        if (strncasecmp(mac_str, mac_prefixes[i], 8) == 0) {
            return true;
        }
    }
    return false;
}

bool check_ssid_pattern(const char* ssid)
{
    if (!ssid) return false;
    for (int i = 0; i < sizeof(wifi_ssid_patterns)/sizeof(wifi_ssid_patterns[0]); i++) {
        if (strcasestr(ssid, wifi_ssid_patterns[i])) {
            return true;
        }
    }
    return false;
}

bool check_device_name_pattern(const char* name)
{
    if (!name) return false;
    
    for (int i = 0; i < sizeof(device_name_patterns)/sizeof(device_name_patterns[0]); i++) {
        if (strcasestr(name, device_name_patterns[i])) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// RAVEN UUID DETECTION
// ============================================================================

// Check if a BLE device advertises any Raven surveillance service UUIDs
bool check_raven_service_uuid(NimBLEAdvertisedDevice* device, char* detected_service_out = nullptr)
{
    if (!device) return false;
    
    // Check if device has service UUIDs
    if (!device->haveServiceUUID()) return false;
    
    // Get the number of service UUIDs
    int serviceCount = device->getServiceUUIDCount();
    if (serviceCount == 0) return false;
    
    // Check each advertised service UUID against known Raven UUIDs
    for (int i = 0; i < serviceCount; i++) {
        NimBLEUUID serviceUUID = device->getServiceUUID(i);
        std::string uuidStr = serviceUUID.toString();
        
        // Compare against each known Raven service UUID
        for (int j = 0; j < sizeof(raven_service_uuids)/sizeof(raven_service_uuids[0]); j++) {
            if (strcasecmp(uuidStr.c_str(), raven_service_uuids[j]) == 0) {
                // Match found! Store the detected service UUID if requested
                if (detected_service_out != nullptr) {
                    strncpy(detected_service_out, uuidStr.c_str(), 40);
                }
                return true;
            }
        }
    }
    
    return false;
}

// Get a human-readable description of the Raven service
const char* get_raven_service_description(const char* uuid)
{
    if (!uuid) return "Unknown Service";
    
    if (strcasecmp(uuid, RAVEN_DEVICE_INFO_SERVICE) == 0)
        return "Device Information (Serial, Model, Firmware)";
    if (strcasecmp(uuid, RAVEN_GPS_SERVICE) == 0)
        return "GPS Location Service (Lat/Lon/Alt)";
    if (strcasecmp(uuid, RAVEN_POWER_SERVICE) == 0)
        return "Power Management (Battery/Solar)";
    if (strcasecmp(uuid, RAVEN_NETWORK_SERVICE) == 0)
        return "Network Status (LTE/WiFi)";
    if (strcasecmp(uuid, RAVEN_UPLOAD_SERVICE) == 0)
        return "Upload Statistics Service";
    if (strcasecmp(uuid, RAVEN_ERROR_SERVICE) == 0)
        return "Error/Failure Tracking Service";
    if (strcasecmp(uuid, RAVEN_OLD_HEALTH_SERVICE) == 0)
        return "Health/Temperature Service (Legacy)";
    if (strcasecmp(uuid, RAVEN_OLD_LOCATION_SERVICE) == 0)
        return "Location Service (Legacy)";
    
    return "Unknown Raven Service";
}

// Estimate firmware version based on detected service UUIDs
const char* estimate_raven_firmware_version(NimBLEAdvertisedDevice* device)
{
    if (!device || !device->haveServiceUUID()) return "Unknown";
    
    bool has_new_gps = false;
    bool has_old_location = false;
    bool has_power_service = false;
    
    int serviceCount = device->getServiceUUIDCount();
    for (int i = 0; i < serviceCount; i++) {
        NimBLEUUID serviceUUID = device->getServiceUUID(i);
        std::string uuidStr = serviceUUID.toString();
        
        if (strcasecmp(uuidStr.c_str(), RAVEN_GPS_SERVICE) == 0)
            has_new_gps = true;
        if (strcasecmp(uuidStr.c_str(), RAVEN_OLD_LOCATION_SERVICE) == 0)
            has_old_location = true;
        if (strcasecmp(uuidStr.c_str(), RAVEN_POWER_SERVICE) == 0)
            has_power_service = true;
    }
    
    // Firmware version heuristics based on service presence
    if (has_old_location && !has_new_gps)
        return "1.1.x (Legacy)";
    if (has_new_gps && !has_power_service)
        return "1.2.x";
    if (has_new_gps && has_power_service)
        return "1.3.x (Latest)";
    
    return "Unknown Version";
}

// ============================================================================
// WIFI PROMISCUOUS MODE HANDLER
// ============================================================================

typedef struct {
    unsigned frame_ctrl:16;
    unsigned duration_id:16;
    uint8_t addr1[6]; /* receiver address */
    uint8_t addr2[6]; /* sender address */
    uint8_t addr3[6]; /* filtering address */
    unsigned sequence_ctrl:16;
    uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
    wifi_ieee80211_mac_hdr_t hdr;
    uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type)
{
    
    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;
    
    // Determine frame subtype from raw packet bytes (more reliable than bitfields)
    const uint8_t* raw = (const uint8_t*)ppkt->payload;
    uint8_t fc0 = raw[0]; // subtype/type in high nibble
    uint8_t fc1 = raw[1]; // flags (ToDS/FromDS etc)
    bool is_probe = (fc0 & 0xF0) == 0x40; // subtype 4 -> 0x40
    bool is_beacon = (fc0 & 0xF0) == 0x80; // subtype 8 -> 0x80
    if (!is_probe && !is_beacon) return;
    
    // Extract SSID from probe request or beacon (robust parser)
    char ssid[33] = {0};

    // Calculate actual header length from fc1 (ToDS/FromDS) using raw bytes
    bool to_ds = (fc1 & 0x01) != 0;
    bool from_ds = (fc1 & 0x02) != 0;
    size_t hdr_len = 24 + ((to_ds && from_ds) ? 6 : 0); // addr4 present when both ToDS and FromDS

    // Pointer to the start of the 802.11 frame (ppkt->payload)
    const uint8_t* pkt_bytes = raw;

    // For beacon frames, skip the fixed parameters (12 bytes) after the header
    size_t ie_offset = hdr_len + (is_beacon ? 12 : 0);

    // Defensive bounds for parsing IEs (we don't have exact packet length here)
    const size_t MAX_IE_PARSE = 512;

    if (ie_offset < MAX_IE_PARSE) {
        const uint8_t* ie_ptr = pkt_bytes + ie_offset;
        size_t idx = 0;
        // Dump small IE area for debugging
        //Serial.println("IE area dump:");
        //dump_packet_hex(ie_ptr, 64);

        while (idx + 1 < MAX_IE_PARSE) {
            uint8_t tag = ie_ptr[idx];
            uint8_t len = ie_ptr[idx + 1];

            // Stop if IE would overflow our parsing window
            if (idx + 2 + (size_t)len > MAX_IE_PARSE) break;

            if (tag == 0) { // SSID element
                size_t copy_len = len > 32 ? 32 : len;
                if (copy_len > 0) memcpy(ssid, &ie_ptr[idx + 2], copy_len);
                ssid[copy_len] = '\0';
                break;
            }

            idx += 2 + (size_t)len;
            if (idx == 0) break; // safety
        }
    }

    const uint8_t* mac = hdr->addr2;
    // Check MAC address
    //char mac_str[9];  // Only need first 3 octets for prefix check
    //snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x", mac[0], mac[1], mac[2]);
    //Serial.println(mac_str);
    // Print SSID for debug (empty means hidden or not present)
    //Serial.print("SSID: ");
    //Serial.println(ssid[0] ? ssid : "<hidden/empty>");
    // Check if SSID matches our patterns
    if (strlen(ssid) > 0 && check_ssid_pattern(ssid)) {
    const char* detection_type = is_probe ? "probe_request" : "beacon";
        output_wifi_detection_json(ssid, hdr->addr2, ppkt->rx_ctrl.rssi, detection_type);
        
        if (!triggered) {
            triggered = true;
            /*
            if(append_mac_to_device_list(mac)){
                flock_detected_beep_sequence(4);
                printf("New device detected \n");
            }
            else{
                flock_detected_beep_sequence(2);
                printf("Device already in detected list, no beep.\n");
            }
            */
            flock_detected_beep_sequence(append_mac_to_device_list(mac));
        }
        // Always update detection time for heartbeat tracking
        last_detection_time = millis();
        return;
    }

    if (check_mac_prefix(hdr->addr2)) {
    const char* detection_type = is_probe ? "probe_request_mac" : "beacon_mac";
        output_wifi_detection_json(ssid[0] ? ssid : "hidden", hdr->addr2, ppkt->rx_ctrl.rssi, detection_type);
        
        if (!triggered) {
            triggered = true;
            /*
            if(append_mac_to_device_list(mac)){
                flock_detected_beep_sequence(4);
                printf("New device detected \n");
            }
            else{
                flock_detected_beep_sequence(2);
                printf("Device already in detected list, no beep.\n");
            }
            */
            flock_detected_beep_sequence(append_mac_to_device_list(mac));
            
        }
        // Always update detection time for heartbeat tracking
        last_detection_time = millis();
        return;
    }
}

// ============================================================================
// BLE SCANNING
// ============================================================================

class AdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        
        NimBLEAddress addr = advertisedDevice->getAddress();
        std::string addrStr = addr.toString();
        uint8_t mac[6];
        sscanf(addrStr.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x", 
               &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
        
        int rssi = advertisedDevice->getRSSI();
        std::string name = "";
        if (advertisedDevice->haveName()) {
            name = advertisedDevice->getName();
        }
        
        // Check MAC prefix
        if (check_mac_prefix(mac)) {
            output_ble_detection_json(addrStr.c_str(), name.c_str(), rssi, "mac_prefix");
            if (!triggered) {
                triggered = true;
                /*
                if(append_mac_to_device_list(mac)){
                    flock_detected_beep_sequence(4);
                    printf("New device detected \n");
                }
                else{
                    flock_detected_beep_sequence(2);
                    printf("Device already in detected list, no beep.\n");
                }
                */
                flock_detected_beep_sequence(append_mac_to_device_list(mac));
            }
            // Always update detection time for heartbeat tracking
            last_detection_time = millis();
            return;
        }
        
        // Check device name
        if (!name.empty() && check_device_name_pattern(name.c_str())) {
            output_ble_detection_json(addrStr.c_str(), name.c_str(), rssi, "device_name");
            if (!triggered) {
                triggered = true;
                /*
                if(append_mac_to_device_list(mac)){
                    flock_detected_beep_sequence(4);
                    printf("New device detected \n");
                }
                else{
                    flock_detected_beep_sequence(2);
                    printf("Device already in detected list, no beep.\n");
                }
                */
                flock_detected_beep_sequence(append_mac_to_device_list(mac));
            }
            // Always update detection time for heartbeat tracking
            last_detection_time = millis();
            return;
        }
        
        // Check for Raven surveillance device service UUIDs
        char detected_service_uuid[41] = {0};
        if (check_raven_service_uuid(advertisedDevice, detected_service_uuid)) {
            // Raven device detected! Get firmware version estimate
            const char* fw_version = estimate_raven_firmware_version(advertisedDevice);
            const char* service_desc = get_raven_service_description(detected_service_uuid);
            
            // Create enhanced JSON output with Raven-specific data
            StaticJsonDocument<1024> doc;
            doc["protocol"] = "bluetooth_le";
            doc["detection_method"] = "raven_service_uuid";
            doc["device_type"] = "RAVEN_GUNSHOT_DETECTOR";
            doc["manufacturer"] = "SoundThinking/ShotSpotter";
            doc["mac_address"] = addrStr.c_str();
            doc["rssi"] = rssi;
            doc["signal_strength"] = rssi > -50 ? "STRONG" : (rssi > -70 ? "MEDIUM" : "WEAK");
            
            if (!name.empty()) {
                doc["device_name"] = name.c_str();
            }
            
            // Raven-specific information
            doc["raven_service_uuid"] = detected_service_uuid;
            doc["raven_service_description"] = service_desc;
            doc["raven_firmware_version"] = fw_version;
            doc["threat_level"] = "CRITICAL";
            doc["threat_score"] = 100;
            
            // List all detected service UUIDs
            if (advertisedDevice->haveServiceUUID()) {
                JsonArray services = doc.createNestedArray("service_uuids");
                int serviceCount = advertisedDevice->getServiceUUIDCount();
                for (int i = 0; i < serviceCount; i++) {
                    NimBLEUUID serviceUUID = advertisedDevice->getServiceUUID(i);
                    services.add(serviceUUID.toString().c_str());
                }
            }
            
            // Output the detection
            serializeJson(doc, Serial);
            Serial.println();
            
            if (!triggered) {
                triggered = true;
                /*
                if(append_mac_to_device_list(mac)){
                    flock_detected_beep_sequence(4);
                    printf("New device detected. \n");
                }
                else{
                    flock_detected_beep_sequence(2);
                    printf("Device already in detected list.\n");
                }
                */
                flock_detected_beep_sequence(append_mac_to_device_list(mac));
            }
            // Always update detection time for heartbeat tracking
            last_detection_time = millis();
            return;
        }
    }
};

// ============================================================================
// CHANNEL HOPPING
// ============================================================================

void hop_channel()
{
    unsigned long now = millis();
    if (now - last_channel_hop > CHANNEL_HOP_INTERVAL) {
        current_channel++;
        if (current_channel > MAX_CHANNEL) {
            current_channel = 1;
        }
        esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
        last_channel_hop = now;
         printf("[WiFi] Hopped to channel %d\n", current_channel);
    }
}



// ============================================================================
// ANIMATION FUNCTION
// ============================================================================
void animate_oled(){
    if(!device_in_range && !triggered){
        u8g2.clearBuffer(); // clear the internal memory
        u8g2.setCursor(xOffset, yOffset+34);
        u8g2.setFont( u8g2_font_5x8_tf);
        if(animframe<3){
            u8g2.drawXBMP(xOffset-5, yOffset, 32, 24, fly00_bitmap);
            u8g2.printf("Scan..");
        } else if (animframe<6){
            u8g2.drawXBMP(xOffset-5, yOffset, 32, 24, fly01_bitmap);
            u8g2.printf("Scan...");
        } else if (animframe<9){
            u8g2.drawXBMP(xOffset-5, yOffset, 32, 24, fly02_bitmap);
            u8g2.printf("Scan");
        } else if (animframe<=11){
            u8g2.drawXBMP(xOffset-5, yOffset, 32, 24, fly03_bitmap);
            u8g2.printf("Scan.");
            if(animframe==11){
                animframe=0;
            }
        }
        // Format the current_action into a C-string and use that for width calculation / printing
        char action_buf[32];
        snprintf(action_buf, sizeof(action_buf), "[%s]", current_action.c_str());
        u8g2.setCursor(xOffset+70 - u8g2.getStrWidth(action_buf), yOffset+34);
        u8g2.printf("%s", action_buf);
        animframe++;
        u8g2.setFont(u8g2_font_u8glib_4_tf);
        u8g2.setCursor(xOffset+20, yOffset+5);
        if(detected_device_count==0){
            u8g2.printf("%d Found (Yay!)", detected_device_count);
        }
        else{
            u8g2.printf("%d Found", detected_device_count);
        }
        u8g2.setCursor(xOffset+25, yOffset+10);
        u8g2.printf("%d Saved", saved_device_count);
        u8g2.setCursor(xOffset+30, yOffset+15);
        u8g2.printf("%d New", new_device_count);
        u8g2.setCursor(xOffset+25, yOffset+21);
        {
            unsigned long up_ms = millis();
            unsigned long up_s = up_ms / 1000;
            unsigned int hours = up_s / 3600;
            unsigned int minutes = (up_s % 3600) / 60;
            unsigned int seconds = up_s % 60;
            u8g2.printf("UP: %02u:%02u:%02u", hours, minutes, seconds);
        }
        u8g2.setCursor(xOffset+20, yOffset+26);
        {
            unsigned long up_ms = millis()-last_detection_time;
            unsigned long up_s = up_ms / 1000;
            unsigned int hours = up_s / 3600;
            unsigned int minutes = (up_s % 3600) / 60;
            unsigned int seconds = up_s % 60;
            if(last_detection_time==0){
                u8g2.printf("LAST: --:--:--");
            }
            else{
            u8g2.printf("LAST: %02u:%02u:%02u", hours, minutes, seconds);
            }
        }
        u8g2.sendBuffer(); // transfer internal memory to the display
    }
}
// ============================================================================
// MAIN FUNCTIONS
// ============================================================================


void setup()
{
    Serial.begin(115200);
    
    // Initialize buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    //delay(5000);
 
    //more oled shenanigans
    
    u8g2.begin();
    u8g2.setContrast(255); // set contrast to maximum 
    u8g2.setBusClock(400000); //400kHz I2C 
    u8g2.setFont(u8g2_font_u8glib_4_tf);
    u8g2.clearBuffer(); // clear the internal memory
    u8g2.drawXBMP(xOffset, yOffset, 70, 40, bird_bitmap);
    u8g2.sendBuffer(); // transfer internal memory to the display
    delay(500);
    boot_beep_sequence();
    delay(1000);

    //init eeprom and load known MAC suffixes
    // Initialize EEPROM and load saved 3-byte MAC suffixes into saved_devices
    {
        // Allocate EEPROM space: 2 bytes for count + 3 bytes per possible saved suffix
        int eepromSize = 2 + MAX_MAC_PATTERNS * 3;
        if (eepromSize < 512) eepromSize = 512; // ensure a reasonable minimum for ESP32
        EEPROM.begin(eepromSize);

        // Read saved device count (stored as little-endian uint16 at addr 0..1)
        uint16_t count = (uint16_t)EEPROM.read(0) | ((uint16_t)EEPROM.read(1) << 8);
        if (count > MAX_MAC_PATTERNS) count = 0; //Set count to 0 in case of new eeprom
        if(WIPE_EEPROM_ON_STARTUP){
            count = 0;
            EEPROM.write(0, 0);
            EEPROM.write(1, 0);
            EEPROM.commit();
            printf("Erasing MAC suffix list...\n");
        }
        saved_device_count = count;

        if (saved_device_count == 0) {
            printf("No saved MAC suffixes in EEPROM\n");
        } else {
            printf("Loading %d saved MAC suffix(es) from EEPROM\n", saved_device_count);
            for (int i = 0; i < saved_device_count; ++i) {
                int base = 2 + i * 3;
                uint8_t b0 = EEPROM.read(base + 0);
                uint8_t b1 = EEPROM.read(base + 1);
                uint8_t b2 = EEPROM.read(base + 2);

                // Store suffix in bytes [3..5] to match runtime checks (first 3 bytes left zero)
                saved_devices[i][0] = 0x00;
                saved_devices[i][1] = 0x00;
                saved_devices[i][2] = 0x00;
                saved_devices[i][3] = b0;
                saved_devices[i][4] = b1;
                saved_devices[i][5] = b2;

                printf("Loaded suffix #%d: %02x:%02x:%02x\n", i, b0, b1, b2);
            }
        }
    }
    delay(1000);
    
    printf("Starting Flock Squawk Enhanced Detection System...\n\n");
    
    // Initialize WiFi in promiscuous mode
    //WiFi.mode(WIFI_STA);
    //WiFi.disconnect();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); 
    esp_wifi_init(&cfg); 
    esp_wifi_set_storage(WIFI_STORAGE_RAM); 
    esp_wifi_set_mode(WIFI_MODE_NULL); 
    esp_wifi_start(); 
    esp_wifi_set_promiscuous(true); 
    delay(100);
    
    esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
    esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
    
    printf("WiFi promiscuous mode enabled on channel %d\n", current_channel);
    printf("Monitoring probe requests and beacons...\n");
    
    // Initialize BLE
    printf("Initializing BLE scanner...\n");
    NimBLEDevice::init("");
    pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    
    printf("BLE scanner initialized\n");
    printf("System ready - hunting for Flock Safety devices...\n\n");
    digitalWrite(LED_PIN, HIGH);
    
    last_channel_hop = millis();
}

void loop()
{
    // Handle channel hopping for WiFi promiscuous mode
    hop_channel();
    current_action = ("CH" + String(current_channel)).c_str();
    // Handle heartbeat pulse if device is in range
    if (device_in_range) {
        unsigned long now = millis();
        
        // Check if 10 seconds have passed since last heartbeat
        if (now - last_heartbeat >= 10000) {
            heartbeat_pulse();
            last_heartbeat = now;
        }
        
        // Check if device has gone out of range (no detection for 30 seconds)
        if (now - last_detection_time >= 30000) {
            printf("Device out of range - stopping heartbeat\n");
            device_in_range = false;
            triggered = false; // Allow new detections
        }
    }
    
    if (millis() - last_ble_scan >= BLE_SCAN_INTERVAL && !pBLEScan->isScanning()) {
        printf("[BLE] scan...\n");
        current_action = String("BLE");
        animate_oled();
        pBLEScan->start(BLE_SCAN_DURATION, false);
        last_ble_scan = millis();
    }
    else animate_oled();
    
    if (pBLEScan->isScanning() == false && millis() - last_ble_scan > BLE_SCAN_DURATION * 1000) {
        pBLEScan->clearResults();
    }
    
    
    delay(50);
    
    /*
    //the oled finale
    u8g2.clearBuffer(); // clear the internal memory
    u8g2.drawFrame(xOffset, yOffset, width, height); //draw a frame around the border
    u8g2.setCursor(xOffset+1, yOffset+12);
    u8g2.printf("%dx%d", width, height);
    u8g2.sendBuffer(); // transfer internal memory to the display
    */
}
