/**
  SePoLab SMART Devices PROJECT
  Name: LEDMatrix_Mesh_node

  Purpose: simulate for Bicycle Signals: Turn Left -Right - Braked

  General Hardware:
  - BASE:
  -- WeMosD1mini/WeMosD1miniPro(required if supported upgrade firmware via OTA) / WeMosD1Lite (no OTA support)
  - SHIELD:
  -- DYI JoyStick shield (required) - used Port: D3;A0;D6;D8
  -- LED Matrix shield (required) - used Port: D5; D7
  
  - CONNECTION:
  -- Upload code for all nodes, no distinct from Joystick or LED Matrix
  -- Each Node can attach Joystick or Matrix shield only; or both

  Pin Function  ESP-8266 Pin
TX  TXD TXD
RX  RXD RXD
A0  Analog input, max 3.3V input  A0
D0  IO  GPIO16
D1  IO, SCL GPIO5
D2  IO, SDA GPIO4
D3  IO, 10k Pull-up GPIO0
D4  IO, 10k Pull-up, BUILTIN_LED  GPIO2
D5  IO, SCK GPIO14
D6  IO, MISO  GPIO12
D7  IO, MOSI  GPIO13
D8  IO, 10k Pull-down, SS GPIO15
G Ground  GND
5V  5V  -
3V3 3.3V  3.3V
RST Reset RST

  General Sofware:
  - 

  @author Sebastian sepolab@gmail.com
  @version 1.0 8/15/17
########################################
*/
#include <painlessMesh.h>
#include <WEMOS_Matrix_LED.h>
// some gpio pin that is connected to an LED...
// on my rig, this is 5, change to the right number of your LED.
#define   LED   D4                 // GPIO number of connected LED, ON WeMos Mini IS D3

#define   BLINK_PERIOD    3000000 // microseconds until cycle repeat
#define   BLINK_DURATION  100000  // microseconds LED is on for

#define   MESH_SSID       "WhatIsTheNode"
#define   MESH_PASSWORD   "WhatIsTheNode"
#define   MESH_PORT       5555

MLED mled(0); //set intensity=0
int buzzer = D8; //Buzzer control port, default D8
unsigned long previousMillis1 = 0; //delay for matrix LED
int previousXValue = 0;
bool buttonRelease = false;
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 500;    // the debounce time; increase if the output flickers
int switchMode = 0;
int temptSwitchMode = 0;
int buzzerPlay = 0;
int z = 0;
int xPin = A0;
int yPin = D3;
int xPosition = 0;
int yPosition = 0;
int buttonStateJoy = 0;
int buttonPin = D6;
char globalMessage[15] = "switchMode = 0";

painlessMesh  mesh;
bool calc_delay = false;
SimpleList<uint32_t> nodes;

void sendMessage() ; // Prototype
Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage ); // start with a one second interval

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes(ERROR | DEBUG);  // set before init() so that you can see startup messages

  mesh.init(MESH_SSID, MESH_PASSWORD, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);

  mesh.scheduler.addTask( taskSendMessage );
  taskSendMessage.enable() ;

  randomSeed(analogRead(A0));

  Serial.begin(115200);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  pinMode(yPin, INPUT_PULLUP);
  pinMode(xPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {

  mesh.update();

  // run the blinky
  bool  onFlag = true;
  uint32_t cycleTime = mesh.getNodeTime() % BLINK_PERIOD;
  for (uint8_t i = 0; i < (mesh.getNodeList().size() + 1); i++) {
    uint32_t onTime = BLINK_DURATION * i * 2;

    if (cycleTime > onTime && cycleTime < onTime + BLINK_DURATION)
      onFlag = false;
  }
  digitalWrite(LED, onFlag);

  if (switchMode ==1) {

    turnRightNEW(3,51);
  } else if (switchMode ==2) {
    turnLeftNEW(3, 51);
  } else if (switchMode ==0) {
    deActiveBrake();
  } else if (switchMode == 8) {
    activeBrake(7);
  }
  delay(50);
  readingJoyStick ();
}

void sendMessage() {
  String msg = globalMessage;
  bool error = mesh.sendBroadcast(msg);
  Serial.print("startHere: Sent msg=");
  Serial.println(msg);
  if (calc_delay) {
    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      mesh.startDelayMeas(*node);
      node++;
    }
    calc_delay = false;
  }

  taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));  // between 1 and 5 seconds
}


void receivedCallback(uint32_t from, String & msg) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  char temptString [15];
  msg.toCharArray(temptString, 15);
  switchMode = temptString[13] - '0';
  snprintf(globalMessage, 15,"switchMode = %d",switchMode);
  deActiveBrake();
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections %s\n", mesh.subConnectionJson().c_str());

  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");

  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
    Serial.printf(" %u", *node);
    node++;
  }
  Serial.println();
  calc_delay = true;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}

void turnRightNEW (int customIntensity, int customSpeed) {
    mled.intensity=customIntensity;//change intensity
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot((y+1)-z-x,4+y); // draw dot
          mled.dot((y+1)-z-x,4-y); // draw dot
        }
      }
      mled.display();
      unsigned long currentMillis1 = millis();
      if (currentMillis1 - previousMillis1 > customSpeed) {
        previousMillis1 = currentMillis1;
        for (int x=0;x<2;x++) {
          for (int y=3;y>=0;y--) {
            mled.dot((y+1)-z-x,4+y,0); // clean dot
            mled.dot((y+1)-z-x,4-y,0); // clean dot
          }
        }
        z++;
        if ( z == 8) { z=0;}
      }
}

void turnLeftNEW (int customIntensity, int customSpeed) {
    mled.intensity=customIntensity;//change intensity
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot((3-y)+z+x,4+y); // draw dot
          mled.dot((3-y)+z+x,4-y); // draw dot
        }
      }
      mled.display();
      unsigned long currentMillis1 = millis();
      if (currentMillis1 - previousMillis1 > customSpeed) {
        previousMillis1 = currentMillis1;
        for (int x=0;x<2;x++) {
          for (int y=3;y>=0;y--) {
            mled.dot((3-y)+z+x,4+y,0); // clean dot
            mled.dot((3-y)+z+x,4-y,0); // clean dot
          }
        }
        z++;
        if ( z == 8) { z=0;}
      }
}

void turnUpNew (int customIntensity, int customSpeed) {
    mled.intensity=customIntensity;//change intensity

      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot(4+y,(3-y)+z+x);
          mled.dot(4-y,(3-y)+z+x);
        }
      }
      mled.display();
     unsigned long currentMillis1 = millis();
      if (currentMillis1 - previousMillis1 > customSpeed) {
        previousMillis1 = currentMillis1;
        for (int x=0;x<2;x++) {
          for (int y=3;y>=0;y--) {
            mled.dot(4+y,(3-y)+z+x,0); // clean dot
            mled.dot(4-y,(3-y)+z+x,0); // clean dot
          }
        }
        z++;
        if ( z == 8) { z=0;}
      }
}

void turnDownNew (int customIntensity, int customSpeed) {
    mled.intensity=customIntensity;//change intensity
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot(4+y,(y+1)-z-x); // draw dot
          mled.dot(4-y,(y+1)-z-x); // draw dot
        }
      }
      mled.display();
     unsigned long currentMillis1 = millis();
      if (currentMillis1 - previousMillis1 > customSpeed) {
        previousMillis1 = currentMillis1;
        for (int x=0;x<2;x++) {
          for (int y=3;y>=0;y--) {
            mled.dot(4+y,(y+1)-z-x,0); // clean dot
            mled.dot(4-y,(y+1)-z-x,0); // clean dot
          }
        }
        z++;
        if ( z == 8) { z=0;}
      }
}

void activeBrake (int customIntensity) {
    mled.intensity=customIntensity;//change intensity
      for (int x=0;x<8;x++) {
        for (int y=0;y<8;y++) {
          mled.dot(x,y); // draw dot
        }
      }
      mled.display();
}

void deActiveBrake () {
      for (int x=0;x<8;x++) {
        for (int y=0;y<8;y++) {
          mled.dot(x,y,0); // clean dot
        }
      }
      mled.display();
}
void readingJoyStick () {
    if ((xPosition <= 320) and buttonRelease) {
      if (switchMode == 2) {
        switchMode = 0;
        snprintf(globalMessage, 15,"switchMode = %d",switchMode);
        sendMessage();
      } else {
        switchMode = 1;
        snprintf(globalMessage, 15,"switchMode = %d",switchMode);
        sendMessage();
      }
    deActiveBrake();
  }
  if (xPosition >= 640 and buttonRelease) {
    if (switchMode == 1) {
      switchMode = 0;
      snprintf(globalMessage, 15,"switchMode = %d",switchMode);
      sendMessage();
    } else {
      switchMode = 2;
      snprintf(globalMessage, 15,"switchMode = %d",switchMode);
      sendMessage();
    }
    deActiveBrake();
  }
  if (buttonStateJoy == 0) {
     if (switchMode > 0) {
       switchMode = 0;
       snprintf(globalMessage, 15,"switchMode = %d",switchMode);
       sendMessage();
     }
  }
  if (yPosition == 0) {
    if (switchMode < 8) {
      temptSwitchMode = switchMode;
      switchMode = 8;
      snprintf(globalMessage, 15,"switchMode = %d",switchMode);
      sendMessage();
    }
  }
   else if (xPosition >= 100) {
    deActiveBrake();
    if (switchMode == 8) {
      switchMode = temptSwitchMode;
      snprintf(globalMessage, 15,"switchMode = %d",switchMode);
      sendMessage();
    }
  }

  xPosition = analogRead(xPin);
  yPosition = digitalRead(yPin);

  float checkPosition = previousXValue - xPosition;
  if (abs(checkPosition) > 150) {
    buttonRelease = true;
  } else {
    buttonRelease = false;
  }

  previousXValue = xPosition;
  buttonStateJoy = digitalRead(buttonPin);
}
