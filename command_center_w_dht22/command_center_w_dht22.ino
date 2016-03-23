// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Please use an Arduino IDE 1.6.8 or greater

#include <ESP8266WiFi.h>
#include <time.h>
#include "dht22.h"
#include "command_center_http.h"


//const char ssid[] = "[SSID]"; //  your WiFi SSID (name)
//const char pass[] = "[PASSWORD]";    // your WiFi password (use for WPA, or use as key for WEP)
//const char connectionString = "HostName=[HubName].azure-devices.net;DeviceId=[DeviceName];SharedAccessKey=[KEY]";
const char ssid[] = "AttunixGuest";
const char pass[] = "a77un!><";
const char connectionString[] = "HostName=RaspPi47914.azure-devices.net;DeviceId=raspy;SharedAccessKey=R3YaruxyEQ0eCi/JXglY1A==";
int status = WL_IDLE_STATUS;

///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    initSerial();
    initDht();
    initWifi();
    initTime();

    if (cmd_ctr_set_connection_string(connectionString)) {
      Serial.print("Connection string successfully set to \"");
      Serial.print(connectionString);
      Serial.println("\"");
    }
    else {
      Serial.println("Unable to set connection string. (too long?)");
    }

    int Init_result__i = cmd_ctr_http_init();
    if (Init_result__i < CMD_CTR_HTTP_INIT_SUCCESS)
    {
      Serial.println("Unable to initialize the Azure connection. Halting.");
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// The Arduino millisecond clock is used here to get a simplistic multitasking effect.
// This macro handles wrap-around correctly.
#define IS_TASK_TIME(curr_time_ms, next_task_time_ms) \
  ((curr_time_ms >= next_task_time_ms) && ( ! ((curr_time_ms ^ next_task_time_ms) & 0x80000000L)))
uint32_t const Sensor_read_period_ms__u32 = 5000;
uint32_t Sensor_read_next_ms__u32 = 0;
uint32_t const Azure_io_update_period_ms__u32 = 100;
uint32_t Azure_io_update_next_ms__u32 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  uint32_t Curr_time_ms__u32 = millis();

  if (IS_TASK_TIME(Curr_time_ms__u32, Sensor_read_next_ms__u32)) {
    float Temp_c__f, Humi_pct__f;
    getNextSample(&Temp_c__f, &Humi_pct__f);
    printf("Temp=%.2f, Pres=%.2f, Humi=%.2f\n", Temp_c__f, Humi_pct__f);
    cmd_ctr_http_send_data(Temp_c__f, Humi_pct__f);

    Sensor_read_next_ms__u32 = Curr_time_ms__u32 + Sensor_read_period_ms__u32;
  }
  
  if (IS_TASK_TIME(Curr_time_ms__u32, Azure_io_update_next_ms__u32)) {
    cmd_ctr_http_run();

    Azure_io_update_next_ms__u32 = Curr_time_ms__u32 + Azure_io_update_period_ms__u32;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void initSerial() {
    // Start serial and initialize stdout
    Serial.begin(115200);
    Serial.setDebugOutput(true);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void initWifi() {
    // Attempt to connect to Wifi network:
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("Connected to wifi");
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void initTime() {
    time_t epochTime;

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    while (true) {
        epochTime = time(NULL);

        if (epochTime == 0) {
            Serial.println("Fetching NTP epoch time failed! Waiting 2 seconds to retry.");
            delay(2000);
        } else {
            Serial.print("Fetched NTP epoch time is: ");
            Serial.println(epochTime);
            break;
        }
    }
}

