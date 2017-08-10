#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <FS.h>
#include <PubSubClient.h> //April23
#include "math.h"
//-------------------------------
//ESP PIN LAYOUT
// SOFT RESET BUTTON: GPIO 0 => use to reset WiFi or MQTT server connection
// LED INDICATER: GPIO 1 => use to inform status of Connection
// SWITCH 1 CONTROL PIN: GPIO 12
// SWITCH 2 CONTROL PIN: GPIO 13
// TOUCH 1 CONTROL PIN: GPIO 15
// TOUCH 2 CONTROL PIN: GPIO 16
// PUBLISH to Trihome/indoor/bedroom1/sub
//  COMMAND = SETCONTROL1 ON/OFF
//  COMMAND = SETCONTROL2 ON/OFF
//---------SOFT RESET BUTTON PARAMETERS---------------------
const int SOFT_RST_PIN =  0;      // SOFT RESET button is map to port 0
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 1000;    // the debounce time; increase if the output flickers

//--------LED INDICATION PARAMETERS--------------------
const int ledPin =  1; // the number of the LED pin
// Variables will change :
int ledState = HIGH;             // ledState used to set the LED
int configureState = 1;   //It is flag to recognize when is in configuration

unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long previousMillis1 = 0;       // will store last time LED was updated
unsigned long previousMillis2 = 0;
unsigned long previousMillis5 = 0;

// constants won't change :
const long interval1000 = 1000;           // interval at which to blink (milliseconds)
const long interval50 = 50;           // interval at which to blink (milliseconds)
const long interval100 = 100;           // interval at which to blink (milliseconds)
const long interval10000 = 10000;           // interval at which to blink (milliseconds)
const long interval300 = 300;           // interval at which to blink (milliseconds)
const long interval5000 = 5000;           // interval at which to blink (milliseconds)

//----------------WIFI CHECK STATE PAMAMETERS----------------
int check = 0;
bool checkWifi = false; //wifi connect indication 0: disconnect; 1: connected
bool wifiReconnecting = false;

//------------WIFI CONFIGURATION PARAMETERS--------------------
bool factoryReset = false;
char APssid[20] = "SePoLab"; //AP is ESP will connect
char APpassword[20] = "tn17691510 ";
const char ssid[] = "TRI02"; // ESP in AP mode
const char password[] = "password";
WiFiClient espClient; //April23
//--------------HTTP client PRAMETERS--------------
WiFiClient webClient;
const char* host = "dweet.io";
const int httpPort = 80;
unsigned long previousMillis9 = 0;
unsigned long previousMillis10 = 0;

//--------------MQTT client PRAMETERS--------------
//define mqtt server default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "192.168.1.30"; //April24
char mqtt_port[6] = "1883"; //April24
char blynk_token[34] = "YOUR_BLYNK_TOKEN";
//flag for saving data
bool shouldSaveConfig = false;
PubSubClient client(espClient);//April23
unsigned long previousMillis3 = 0; //set time for interval publish
unsigned long previousMillis4 = 0; //set time for interval publish
//char mqttClientID[] = ssid;
char pubTopicGen[40] = "Trihome/tri02/state";
char subTopicName[40] = "Trihome/tri02/set";
int valueOfSensor = 0;// value that will be published
char pubMsg[80] = "Hello Server,it is client's 1st message!";//payload of publishing message
char defaultMsg[] = "Hello its default message payload";//payload of publishing message
char subMsg[50]; //payload of subcribled message
long publishInveral = 15000;
bool mqttConnected = false;
const long reconnectInveral = 10000;
char isON[] = "F";
char isSwitch[] = "F";
char isOpen[] = "F";
bool firstTime = true;
//----------ESP API in case using DTH21 CONFIGURATION---------------
bool isDefinedCommand = true;
bool simulatedDropedWifi = false;
//---------CONTROL CONFIGURATION---------------------------------------
int pinControl1 = 12;           // Use pinLED of Wifi indication.
int pinControl1State = HIGH;
int touch1 = 15;
unsigned long previousMillis6 = 0;
unsigned long previousMillis7 = 0;
int relayCriteria = 3;
float minIn = 29;
float maxIn = 31;
//---------CONTROL CONFIGURATION---------------------------------------
int pinControl2 = 13;
int pinControl2State = HIGH;
int touch2 = 16;
unsigned long previousMillis11 = 0;
unsigned long previousMillis12 = 0;
//--------END VAR=-----------------------------------------------
//---------TOUCH1 CONFIGURATION---------------------------------------
int lasttouch1ReadingState = LOW;
long touch1DebounceTime = 0;
int touch1State = LOW;
bool isPressTouch1 = false;
//---------TOUCH2 CONFIGURATION---------------------------------------
int lasttouch2ReadingState = LOW;
long touch2DebounceTime = 0;
int touch2State = LOW;
bool isPressTouch2 = false;
bool sendOnetime1 = true; // for 1st time startup, send to broker to reset the status
bool sendOnetime2 = true; // for 1st time startup, send to broker to reset the status
//------------------------------------------------
//void API for cloud command

// void is use to INIT/STOP RELAY with mqtt command
void setControl1(char inputString[]) {
  char breakedCommand[] = "SETCONTROL1";
  char breakedValue[10];
  int check = 0;
  int countSpace = 0;
  int i = 0;
  int j = 0;
  int countOfChar = 0;
  while (inputString[i] != '#') {
    if (inputString[i] != ' ') {
        if (countSpace == 0) {
            if (breakedCommand[i] != inputString[i]) {
                check++;
            }
            countOfChar++;
        } else {
                breakedValue[j] = inputString[i];
                j++;
            }
      } else {
      countSpace++;
        }
    i++;
  }
  if ((countOfChar == (sizeof(breakedCommand)-1)) and (check == 0)) {
        if ((breakedValue[0] == 'O') and (breakedValue[1] == 'N') and (j == 2)) {
          pinControl1State = LOW;
          digitalWrite(pinControl1, pinControl1State);
          if ((!firstTime) and (client.connected())) {
              snprintf (isON, 4, "ON");
              snprintf(pubMsg, 80, "SETCONTROL1 %s",isON);
              client.publish(pubTopicGen, pubMsg,true);
              delay(50);
          }
        }
        else if ((breakedValue[0] == 'O') and (breakedValue[1] == 'F') and (breakedValue[2] == 'F') and (j == 3)) {
          pinControl1State = HIGH;
          digitalWrite(pinControl1, pinControl1State);
            if ((!firstTime) and client.connected()) {
              snprintf (isON, 4, "OFF");
              snprintf(pubMsg, 80, "SETCONTROL1 %s",isON);
              client.publish(pubTopicGen, pubMsg,true);
              delay(50);
          }
        }
      isDefinedCommand = true;
      }
     else {
      isDefinedCommand = false;
    }
}

// void is use to INIT/STOP RELAY with mqtt command
void setControl2(char inputString[]) {
  char breakedCommand[] = "SETCONTROL2";
  char breakedValue[10];
  int check = 0;
  int countSpace = 0;
  int i = 0;
  int j = 0;
  int countOfChar = 0;
  while (inputString[i] != '#') {
    if (inputString[i] != ' ') {
        if (countSpace == 0) {
            if (breakedCommand[i] != inputString[i]) {
                check++;
            }
            countOfChar++;
        } else {
                breakedValue[j] = inputString[i];
                j++;
            }
      } else {
      countSpace++;
        }
    i++;
  }
  if ((countOfChar == (sizeof(breakedCommand)-1)) and (check == 0)) {
        if ((breakedValue[0] == 'O') and (breakedValue[1] == 'N') and (j == 2)) {
          pinControl2State = LOW;
          digitalWrite(pinControl2, pinControl2State);
          if ((!firstTime) and (client.connected())) {
              snprintf (isSwitch, 4, "ON");
              snprintf(pubMsg, 80, "SETCONTROL2 %s",isSwitch);
              client.publish(pubTopicGen, pubMsg,true);
              delay(50);
          }
        }
        else if ((breakedValue[0] == 'O') and (breakedValue[1] == 'F') and (breakedValue[2] == 'F') and (j == 3)) {
          pinControl2State = HIGH;
          digitalWrite(pinControl2, pinControl2State);
            if ((!firstTime) and client.connected()) {
              snprintf (isSwitch, 4, "OFF");
              snprintf(pubMsg, 80, "SETCONTROL2 %s",isSwitch);
              client.publish(pubTopicGen, pubMsg,true);
              delay(50);
          }
        }
      isDefinedCommand = true;
      }
     else {
      isDefinedCommand = false;
    }
}
//-------- END case 1.2 if NORMAL MODE-------------------------
//-----void callback description----
//If the client is used to subscribe to topics, a callback function must be provided in the constructor.
//This function is called when new messages arrive at the client.
//topic - the topic the message arrived on (const char[])
//payload - the message payload (byte array)
//length - the length of the message payload (unsigned int)
//-----------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < 30; i++) {
    subMsg[i] = '#';
  }
  Serial.print("Message arrived from [");
  Serial.print(mqtt_server);
  Serial.print("] ;topic: [");
  Serial.print(topic);
  Serial.print("] with payload: [");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    subMsg[i] = (char)payload[i];
  }
  Serial.print("]");
  Serial.println();
////--------detect command-----------
  isDefinedCommand = false;
  if (!isDefinedCommand) { setControl1(subMsg); }
  if (!isDefinedCommand) { setControl2(subMsg); }
  if (!isDefinedCommand) {
      snprintf (pubMsg, 30, "COMMAND NOT FOUND"); //strtok(inputString,"#"));
      Serial.print("Message send: ");
      Serial.println(pubMsg);
      client.publish(pubTopicGen, pubMsg,true);
  }
//----------END case 1.2 of NORMAL MODE -----------------------
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(APssid);
  WiFi.begin(APssid, APpassword);
}

void reconnect() {
  // Loop until we're reconnected
  //  while (!client.connected()) {
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ssid)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      snprintf (pubMsg, 70, "ID: %s; state: Reconnecting...", ssid);
      client.publish(pubTopicGen, pubMsg,true);
      // ... and resubscribe
      client.subscribe(subTopicName);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 10 seconds");
    }
  }
}

//-------end Apirl 23 2016------------



void setup() {
  // set the digital pin as output:
  //SET GPIO 0 IS SOFT RESET PIN AND IS INPUT
  pinMode(SOFT_RST_PIN, INPUT);
  // put your setup code here, to run once:
  //
  Serial.begin(115200);
  //Start DTH
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  //    WiFiManager wifiManager;
  ledState = LOW;
//  digitalWrite(ledPin, ledState);
  pinMode(pinControl1, OUTPUT);
  digitalWrite(pinControl1, pinControl1State);
  pinMode(pinControl2, OUTPUT);
  digitalWrite(pinControl2, pinControl2State);
  pinMode(touch1, INPUT);
  pinMode(touch2, INPUT);

  //  //exit after config instead of connecting
  //  wifiManager.setBreakAfterConfig(true);

  //reset saved settings for testing only
  //    wifiManager.resetSettings();
  //    delay(1500);

  //clean FS, for testing
//    SPIFFS.format();

  //--------Read Congiguration file-----------
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        factoryReset = false;
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed config parameters");
          strcpy(APssid, json["APssid"]);
          strcpy(APpassword, json["APpassword"]);
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(pubTopicGen, json["pubTopicGen"]);
          strcpy(subTopicName, json["subTopicName"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    } else {
      Serial.println("File Config has not created");
      factoryReset = true;
    }
  }
  else {
    Serial.println("failed to mount FS");
  }
  //--------END Read Congiguration file-----------


  //  check wifi status is CONNECTED = 3; other values means disconnected to AP.
  setup_wifi();
  delay(5000);
  //if you get here you have connected to the WiFi
  if (WiFi.status() == WL_CONNECTED) {
    checkWifi = true;
    Serial.println(mqtt_server);
    client.setServer(mqtt_server, atoi(mqtt_port)); //April23
    client.setCallback(callback); //April23
    Serial.println("DONE Setup mqtt sever");
  }
  ledState = HIGH;
//  digitalWrite(ledPin, ledState);
  reconnect();
}


//-------Apirl 23 2016------------

void loop() {

  WiFiManager wifiManager;
  // read the state of the switch into a local variable:
  int reading = digitalRead(SOFT_RST_PIN);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:
  // If the switch changed, due to noise or pressing:

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  //--------RESET WIFI CONFIGURATION-------------------------------
  if ((factoryReset == true) or ((millis() - lastDebounceTime) > debounceDelay)) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    Serial.println("ACCESSING WIFI DIRECT - AP");
    //WiFiManager

    //reset saved settings
    wifiManager.resetSettings();
    delay(2000);
    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep
    //in seconds
    ledState = LOW;
//    digitalWrite(ledPin, ledState);
    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length
    WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
    WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
//    WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 32);
    WiFiManagerParameter AP_password("p", "abcd1234", password, 20);
    WiFiManagerParameter custom_subscribe_topic("subscribe", "subscribe_topic",subTopicName , 40);
    WiFiManagerParameter custom_GeneralPublish_topic("pubTopicGen", "GeneralPublish_topic", pubTopicGen, 40);
    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    //exit after config instead of connecting
    wifiManager.setBreakAfterConfig(true);

    //add all your parameters here
    wifiManager.addParameter(&AP_password);
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
//    wifiManager.addParameter(&custom_blynk_token);
    wifiManager.addParameter(&custom_subscribe_topic);
    wifiManager.addParameter(&custom_GeneralPublish_topic);


    if (!wifiManager.autoConnect(ssid, password)) {
      Serial.println("failed to connect existing SSID ...");
    }
    else {
      ledState = HIGH;
//      digitalWrite(ledPin, ledState);
      //read updated parameters
      WiFi.status();
      strcpy(mqtt_server, custom_mqtt_server.getValue());
      strcpy(mqtt_port, custom_mqtt_port.getValue());
//      strcpy(blynk_token, custom_blynk_token.getValue());
      strcpy(APpassword, AP_password.getValue());
      strcpy(subTopicName, custom_subscribe_topic.getValue());
      strcpy(pubTopicGen, custom_GeneralPublish_topic.getValue());
      //save the custom parameters to FS
      if (shouldSaveConfig) {
        Serial.println("saving config");
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
        json["APssid"] = WiFi.SSID();
        json["APpassword"] = APpassword;
        json["mqtt_server"] = mqtt_server;
        json["mqtt_port"] = mqtt_port;
        json["subTopicName"] = subTopicName;
        json["pubTopicGen"] = pubTopicGen;
        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
          Serial.println("failed to open config file for writing");
        }
        json.printTo(Serial);
        json.printTo(configFile);
        configFile.close();
        Serial.println();
        //end save
      }
      delay(1000);
      ESP.reset();
      delay(5000);
    }
  }
  //-----------END RESET--------------------

  //---START WI4341-----------------------------------------------------
  //Description: Mode Wifi Client : Loop Authen until successful
  if (wifiReconnecting == true) {
    //      if (!wifiManager.autoConnect(SSIDNAME)) {
    if ((simulatedDropedWifi) or (WiFi.status() != WL_CONNECTED)) {
      Serial.println("failed to connect, trying in seconds....");
      checkWifi = false;
    }
    else {
      Serial.print("connected at IP: ");
      Serial.println(WiFi.localIP());
      if (checkWifi == false) {
        WiFiClient espClient;
        WiFiClient webClient;
        webClient.connect(host, httpPort);
        PubSubClient client(espClient);
        client.setServer(mqtt_server, atoi(mqtt_port)); //April23
        client.setCallback(callback); //April23
      }
      checkWifi = true;
    }
  }
  //----STOP WI4341----------------------------------------------------
//Merge to put data both mqtt server and http server
  if (checkWifi == true) {
    unsigned long currentMillis3 = millis();
    if (currentMillis3 - previousMillis3 > publishInveral) {
      previousMillis3 = currentMillis3;
      unsigned long currentMillis4 = millis();
      snprintf (isON, 1, "F");
      snprintf (isSwitch, 1, "F");
      if (client.connected() and webClient.connect(host, httpPort)) {
        firstTime = false;
        //-----LIGHT PUT----BEGIN-----
        if (pinControl1State == LOW) {
            snprintf (isON, 4, "ON");
            if (sendOnetime1 == true) {
              snprintf(pubMsg, 80, "SETCONTROL1 %s",isON);
              client.publish(pubTopicGen, pubMsg,true);
              delay(50);
              sendOnetime1 = false;
            }
        } else if (pinControl1State == HIGH) {
            snprintf (isON, 4, "OFF");
            if (sendOnetime1 == true) {
              snprintf(pubMsg, 80, "SETCONTROL1 %s",isON);
              client.publish(pubTopicGen, pubMsg,true);
              delay(50);
              sendOnetime1 = false;
            }
        }
        //-----LIGHT PUT----END-----
        //-----DOOR SENSOR PUT----BEGIN-----
        if (pinControl2State == LOW) {
            snprintf (isSwitch, 4, "ON");
            if (sendOnetime2 == true) {
              snprintf(pubMsg, 80, "SETCONTROL2 %s",isSwitch);
              client.publish(pubTopicGen, pubMsg,true);
              delay(50);
              sendOnetime2 = false;
            }
        } else if (pinControl2State == HIGH) {
            snprintf (isSwitch, 4, "OFF");
            if (sendOnetime2 == true) {
              snprintf(pubMsg, 80, "SETCONTROL2 %s",isSwitch);
              client.publish(pubTopicGen, pubMsg,true);
              delay(50);
              sendOnetime2 = false;
            }
        }
        //-----DOOR PUT----END-----
    }
    else {
      if ((currentMillis4 - previousMillis4 > reconnectInveral)) {
      previousMillis4 = currentMillis4;
      reconnect();
      }
    }
  }
  client.loop();
  }
  //Read data from touch1 button

int touch1Reading = digitalRead(touch1);
  if (touch1Reading != lasttouch1ReadingState) {
    touch1DebounceTime = millis();
  }

if ((millis() - touch1DebounceTime) > 200) {
    if (touch1Reading != touch1State) {
        touch1State = touch1Reading;
        if (touch1State == HIGH) {
          isPressTouch1 = true;
          Serial.println("isPressTouch1=true");
        }
    }
  }
  if (isPressTouch1) {
      isPressTouch1 = false;
      if (pinControl1State == LOW) { pinControl1State = HIGH;Serial.println("pinControl1State = HIGH");}
      else{ pinControl1State = LOW;Serial.println("pinControl1State = LOW");}
      digitalWrite(pinControl1, pinControl1State);
  //NEW FEATURE: DOOR SENSOR WILL DETECT DOOR OPEN ROOM AND SEE TO SERVER DWEET.IO
//--------Door sensor analysis and check up---------------   sendOnetime1 = true;
    }
  lasttouch1ReadingState = touch1Reading;
//end of Read data from touch1 button

//Read data from touch2 button
int touch2Reading = digitalRead(touch2);
  if (touch2Reading != lasttouch2ReadingState) {
    touch2DebounceTime = millis();
  }

if ((millis() - touch2DebounceTime) > 200) {
    if (touch2Reading != touch2State) {
        touch2State = touch2Reading;
        if (touch2State == HIGH) {
          isPressTouch2 = true;
          Serial.println("isPressTouch2=true");
        }
    }
  }
  if (isPressTouch2) {
      isPressTouch2 = false;
      if (pinControl2State == LOW) { pinControl2State = HIGH;Serial.println("pinControl2State = HIGH");}
      else{ pinControl2State = LOW;Serial.println("pinControl2State = LOW");}
      digitalWrite(pinControl2, pinControl2State);
      sendOnetime2 = true;
    }
  lasttouch2ReadingState = touch2Reading;
//end of Read data from touch2 button

}
