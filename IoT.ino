//#######################################################
//                                                     //
//  Revision 1: 13/08/2019                             //
//             - code created                          //
//                                                     //
//  This code was written by Joe Orford for OKDO       //
//                                                     //
//  Do as you please with the code, design the world   //
//                                                     //
//#######################################################

//This code was written for the MKR Wi-Fi 1010 and hasn't been tested on any other board
//In theory it should work with any Wi-Fi NINA board


//uncomment out the line below if you want to lower the power usage
#define debug_mode

//add your details here
const char* DEVICE_ID = "Your Device ID";
const char* DEVICE_TOKEN = "Your Device Token";


const char* ssid = "MyWifi";
const char* pass = "MyWifiPassword";

#include <WiFiNINA.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <ATT_IOT.h>
#include <SPI.h>  // required to have support for signed/unsigned long type
#include <CborBuilder.h>
#include "ArduinoLowPower.h"
#include "sgp30.h"
#include "keys.h"

int status = WL_IDLE_STATUS;

// Define http and mqtt endpoints
#define httpServer "api.allthingstalk.io"  // API endpoint
#define mqttServer "api.allthingstalk.io"  // broker

ATTDevice device(DEVICE_ID, DEVICE_TOKEN);
CborBuilder payload(device);

WiFiClient client;
PubSubClient pubSub(mqttServer, 1883, client);

void connect_to_wifi() {
#ifdef debug_mode
  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }
#endif

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
#ifdef debug_mode
    Serial.print("[WiFi] Connecting to: ");
    Serial.println(ssid);
#endif

    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 1 second for connection:
    delay(1000);
  }

#ifdef debug_mode
  // you're connected now, so print out the data:
  Serial.println("[WiFi] Connected");
#endif
}

void setup()
{
#ifdef debug_mode
  Serial.begin(115200);       // init serial link for debugging
#endif

  connect_to_wifi();

  while (sgp_probe() != STATUS_OK) {
#ifdef debug_mode
    Serial.println("SGP init failed");
#endif
    while (1);
  }

  delay(1000);  // give the Ethernet shield a second to initialize

  while (!device.connect(&client, httpServer)) { // connect the device with AllThingsTalk
#ifdef debug_mode
    Serial.println("retrying");
#endif
  }
  
  while (!device.subscribe(pubSub)) { // make certain that we can receive messages over mqtt
#ifdef debug_mode
    Serial.println("retrying");
#endif
  }
}

unsigned long prevTime;  // only send every x milliseconds

void loop()
{
  s16 err = 0;
  u16 tvoc_ppb, co2_eq_ppm;

  err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
  if (err != STATUS_OK) {
#ifdef debug_mode
    Serial.println("error reading tvoc_ppb & co2_eq_ppm values\n");
#endif
  }

  unsigned long curTime = millis();
  if (curTime > (prevTime + 5000))  // enough time has passed
  {
    payload.reset();
    payload.map(1);
    payload.addNumber(co2_eq_ppm, "co2");
    payload.send();
    prevTime = curTime;
  }
  device.process();
}
