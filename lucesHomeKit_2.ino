/*
  Internet & Manual Home Automation with Apple Home and Siri
  Original Code created by Mixiaoxiao (Wang Bin) and edited to work with manual switches by Sachin Soni (techiesms)
  To watch out full tutorial video, head on to my YouTube channel
  https://www.YouTube.com/techiesms
       techiesms
  explore | learn | share
*/

#include <Arduino.h>
#include <arduino_homekit_server.h>
#include "wifi_info.h"
#include  "OTA.h"

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);

// Switches & Relay Pin definition
#define S1 2
#define R1 0

#define DEBUG_SW 3

int var1 = 0; // Used for preventing a double-action on the switch feedback function
int var2 = 0; // Same shit as before
int previousp = 1; // Used to register the last known postition of the physical switch
int switch_ON_Flag1_previous_I = 1;


void setup() {
  Serial.begin(115200);
  wifi_connect(); // in wifi_info.h
  setupOTA();
  // homekit_storage_reset(); // to remove the previous HomeKit pairing storage when you first run this new HomeKit example
  my_homekit_setup();
  
}

void loop() {
  
//  while (WiFi.isConnected()) {
    my_homekit_loop();
    delay(10);
   ArduinoOTA.handle();
//    }
//  Serial.println("Wifi disconnected,trying to reconnect...");
//  wifi_connect();
}

//==============================
// HomeKit setup and loop
//==============================

// access your HomeKit characteristics defined in my_accessory.c
extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t cha_switch1_on;

static uint32_t next_heap_millis = 0;

//Called when the switch value is changed by iOS Home APP
void cha_switch1_on_setter(const homekit_value_t value) {
  bool on = value.bool_value;
  cha_switch1_on.value.bool_value = on;  //sync the value
  LOG_D("Switch: %s", on ? "ON" : "OFF");
  digitalWrite(R1, on ? LOW : HIGH);
  switch_ON_Flag1_previous_I = on ? LOW : HIGH;
}

void my_homekit_setup()
{
  pinMode(S1, INPUT);
  pinMode(R1, OUTPUT);

  //Add the .setter function to get the switch-event sent from iOS Home APP.
  //The .setter should be added before arduino_homekit_setup.
  //HomeKit sever uses the .setter_ex internally, see homekit_accessories_init function.
  //Maybe this is a legacy design issue in the original esp-homekit library,
  //and I have no reason to modify this "feature".
  cha_switch1_on.setter = cha_switch1_on_setter;

  arduino_homekit_setup(&config);


}


void my_homekit_loop() {
  arduino_homekit_loop();
  switch_feedback_function();
  const uint32_t t = millis();
  if (t > next_heap_millis) {
    // show heap info every 5 seconds
    next_heap_millis = t + 5 * 1000;
    LOG_D("Free heap: %d, HomeKit clients: %d",
          ESP.getFreeHeap(), arduino_homekit_connected_clients_count());



  }

}
void switch_feedback_function()
{
if (digitalRead(S1) == HIGH )
  {
    if (previousp == 1){
      previousp = 0;

      if (switch_ON_Flag1_previous_I == 1)
      {
        digitalWrite(R1, LOW);
        if (DEBUG_SW) Serial.println("Relay1 OFF");
          //report the switch value to HomeKit if it is changed (e.g. by a physical button)
          bool switch1_is_on = 1;
          cha_switch1_on.value.bool_value = switch1_is_on;
          homekit_characteristic_notify(&cha_switch1_on, cha_switch1_on.value);

          switch_ON_Flag1_previous_I = 0;
          var1 = 1;
      }
      if (switch_ON_Flag1_previous_I == 0 && var1 == 0)
      {
        digitalWrite(R1, HIGH);
        if (DEBUG_SW) Serial.println("Relay1 OFF");
        //report the switch value to HomeKit if it is changed (e.g. by a physical button)
        bool switch1_is_on = 0;
        cha_switch1_on.value.bool_value = switch1_is_on;
        homekit_characteristic_notify(&cha_switch1_on, cha_switch1_on.value);

        switch_ON_Flag1_previous_I = 1;
      }
    }
    var1 = 0;
  }


if (digitalRead(S1) == LOW )
  {
    if (previousp == 0){
previousp = 1;
    if (switch_ON_Flag1_previous_I == 0)
    {
      digitalWrite(R1, HIGH);
      if (DEBUG_SW) Serial.println("Relay1 OFF");
      //report the switch value to HomeKit if it is changed (e.g. by a physical button)
      bool switch1_is_on = 0;
      cha_switch1_on.value.bool_value = switch1_is_on;
      homekit_characteristic_notify(&cha_switch1_on, cha_switch1_on.value);

      switch_ON_Flag1_previous_I = 1;
      var2 = 1;
    }
    if (switch_ON_Flag1_previous_I == 1 && var2 == 0)
    {
      digitalWrite(R1, LOW);
      if (DEBUG_SW) Serial.println("Relay1 OFF");
      //report the switch value to HomeKit if it is changed (e.g. by a physical button)
      bool switch1_is_on = 1;
      cha_switch1_on.value.bool_value = switch1_is_on;
      homekit_characteristic_notify(&cha_switch1_on, cha_switch1_on.value);

      switch_ON_Flag1_previous_I = 0;
    }
    }
    var2 = 0;
  }
}
