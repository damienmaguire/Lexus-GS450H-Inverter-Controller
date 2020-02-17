/*
The information will be provided over serial to the esp8266 at 19200 baud 8n1 in the form :
vxxx,ixxx,pxxx,mxxxx,nxxxx,oxxx,rxxx,qxxx* where :

v=pack voltage (0-700Volts)
i=current (0-1000Amps)
p=power (0-300kw)
m=mg1 rpm (0-10000rpm)
n=mg2 rpm (0-10000rpm)
o=mg1 temp (-20 to 120C)
r=mg2 temp (-20 to 120C)
q=oil pressure (0-100%)
*=end of string
xxx=three digit integer for each parameter eg p100 = 100kw.
updates will be every 100ms approx.

v100,i200,p35,m3000,n4000,o20,r100,q50*

*/

// Import required libraries
#ifdef ESP32
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#else
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#endif
#include <Wire.h>

// declaration of a variable
String v, i, p, m, n, o, r, q;

// Replace with your network credentials
const char* ssid = "";
const char* password = "";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String PackVoltage() {
  return v;
}
String Current() {
  return i;
}
String Power() {
  return p;
}
String mg1RPM() {
  return m;
}
String mg2RPM() {
  return n;
}
String mg1Temp() {
  return o;
}
String mg2Temp() {
  return r;
}
String oilPressure() {
  return q;
}

String getBGcolor() {
  File BGcolor = SPIFFS.open("/BGcolor.txt", "r");
  String value = BGcolor.readString();
  BGcolor.close();
  return (value);
}

String getHeading() {
  File Heading = SPIFFS.open("/Heading.txt", "r");
  String value = Heading.readString();
  Heading.close();
  return (value);
}

String getSsid() {
  File ssid = SPIFFS.open("/ssid.txt", "r");
  String value = ssid.readString();
  ssid.close();
  return (value);
}

String getPassword() {
  File password = SPIFFS.open("/password.txt", "r");
  String value = password.readString();
  password.close();
  return (value);
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(19200);

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  Serial.println(getSsid());
  Serial.println(getPassword());
  //Start WiFi AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(getSsid(), getPassword());

  // Route for root / web pages
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html");
  });
  server.on("/admin", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/admin.html");
  });
  server.on("/highcharts.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/highcharts.js", "text/javascript");
  });
  server.on("/highcharts-more.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/highcharts-more.js", "text/javascript");
  });
  server.on("/solid-gauge.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/solid-gauge.js", "text/javascript");
  });
  server.on("/PackVoltage", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", PackVoltage().c_str());
  });
  server.on("/Current", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", Current().c_str());
  });
    server.on("/Power", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", Power().c_str());
  });
  server.on("/mg1RPM", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", mg1RPM().c_str());
  });
  server.on("/mg2RPM", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", mg2RPM().c_str());
  });
  server.on("/mg1Temp", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", mg1Temp().c_str());
  });
  server.on("/mg2Temp", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", mg2Temp().c_str());
  });
  server.on("/oilPressure", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", oilPressure().c_str());
  });
  server.on("/getBGcolor", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getBGcolor().c_str());
  });
  server.on("/getSsid", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getSsid().c_str());
  });
  server.on("/getPassword", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getPassword().c_str());
  });
  server.on("/getHeading", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getHeading().c_str());
  });
  server.on("/setBGcolor", HTTP_GET, [](AsyncWebServerRequest * request) {
    //Serial.println(request->getParam("favcolor")->value());
    File BGcolor = SPIFFS.open("/BGcolor.txt", "w");
    BGcolor.print(request->getParam("favcolor")->value());
    BGcolor.close();

    File Heading = SPIFFS.open("/Heading.txt", "w");
    Heading.print(request->getParam("heading")->value());
    Heading.close();

    File ssid = SPIFFS.open("/ssid.txt", "w");
    ssid.print(request->getParam("ssid")->value());
    ssid.close();

    File password = SPIFFS.open("/password.txt", "w");
    password.print(request->getParam("password")->value());
    password.close();

    request->redirect("/");
  });

  server.begin();
}

void loop() {
  String justRates = Serial.readStringUntil('\n');
  //split justrate variable from begining to first "," charactor
  int x1stSpaceIndex = justRates.indexOf(",");
  int x2ndSpaceIndex = justRates.indexOf(",", x1stSpaceIndex + 1);
  int x3rdSpaceIndex = justRates.indexOf(",", x2ndSpaceIndex + 1);
  int x4thSpaceIndex = justRates.indexOf(",", x3rdSpaceIndex + 1);
  int x5thSpaceIndex = justRates.indexOf(",", x4thSpaceIndex + 1);
  int x6thSpaceIndex = justRates.indexOf(",", x5thSpaceIndex + 1);
  int x7thSpaceIndex = justRates.indexOf(",", x6thSpaceIndex + 1);
  v = (justRates.substring(1, x1stSpaceIndex)).toInt();
  i = (justRates.substring(x1stSpaceIndex + 2, x2ndSpaceIndex));
  p = (justRates.substring(x2ndSpaceIndex + 2, x3rdSpaceIndex));
  m = (justRates.substring(x3rdSpaceIndex + 2, x4thSpaceIndex));
  n = (justRates.substring(x4thSpaceIndex + 2, x5thSpaceIndex));
  o = (justRates.substring(x5thSpaceIndex + 2, x6thSpaceIndex));
  r = (justRates.substring(x6thSpaceIndex + 2, x7thSpaceIndex));
  q = (justRates.substring(x7thSpaceIndex + 2)).toInt();

  Serial.println();
  Serial.print("Serial input - ");
  Serial.println(justRates);
  Serial.print("Split values - ");
  Serial.println("v : " + String(v) + " i : " + String(i) + " p : " + (p) + " m : " + String(m) + " n : " + String(n) + " o : " + String(o) + " r " + String(r) + " q : " + String(q));
}
