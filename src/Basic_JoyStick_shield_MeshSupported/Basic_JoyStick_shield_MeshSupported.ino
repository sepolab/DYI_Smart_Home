#include <WEMOS_Matrix_LED.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMesh.h>

unsigned int request_i = 0;
unsigned int response_i = 0;

String manageRequest(String request);
/* Create the mesh node object */
ESP8266WiFiMesh mesh_node = ESP8266WiFiMesh(ESP.getChipId(), manageRequest);

MLED mled(0); //set intensity=0
int buzzer = D8; //Buzzer control port, default D8
unsigned long previousMillis1 = 0; //delay for matrix LED
int previousXValue = 0;
bool buttonRelease = false;
const int changeModePIN = D3;
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
int yPin = D4;
int xPosition = 0;
int yPosition = 0;
int buttonStateJoy = 0;
int buttonPin = D6;

void setup() {
  Serial.begin(115200);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  pinMode(yPin, INPUT);
  pinMode(xPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
   
    /* Initialise the mesh node */
  mesh_node.begin();
}

String manageRequest(String request)
{
  /* Print out received message */
  Serial.print("received: ");
  Serial.println(request);
 
  /* return a string to send back */
  char response[60];
  sprintf(response, "OK from Mesh_Node%d.", ESP.getChipId());
  return response;
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
    temptSwitchMode = 1;
  }
  if (xPosition >= 640 and buttonRelease) {
    temptSwitchMode = 2; 
  }
  if (buttonStateJoy == 0) {
     temptSwitchMode = 0;
  }
  
  if (yPosition == 0) {
    activeBrake(7);
    if (switchMode != 9) {
      temptSwitchMode = switchMode;
    }
    switchMode = 9;
  }
   else {
    deActiveBrake();
    switchMode = temptSwitchMode;
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

void loop() {
    /* Accept any incoming connections */
//  mesh_node.acceptRequest();
  char request[60];
  sprintf(request, "%d from Mesh_Node%d", switchMode, ESP.getChipId());
  mesh_node.attemptScan(request);

  Serial.println(request);
  delay(50);
  readingJoyStick ();
}
