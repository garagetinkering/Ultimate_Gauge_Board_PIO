#include "Arduino.h"
#include "CANBus_Driver.h"
#include <stdio.h>

void canbus_init(void) {

  // Configure TWAI (CAN)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();  // Accept all IDs
 
    // Install and start TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI driver installed.");
    } else {
        Serial.println("Failed to install TWAI driver.");
        while (1);
    }

    if (twai_start() == ESP_OK) {
        Serial.println("TWAI driver started. Listening for messages...");
    } else {
        Serial.println("Failed to start TWAI driver.");
        while (1);
    }
}