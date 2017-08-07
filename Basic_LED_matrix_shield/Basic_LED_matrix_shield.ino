#include <WEMOS_Matrix_LED.h>

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
}

void turnLeft (int customIntensity, int customSpeed) {
    mled.intensity=customIntensity;//change intensity
    for (int z=0;z<8;z++){
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot((y+1)-z-x,4+y); // clean dot
          mled.dot((y+1)-z-x,4-y); // clean dot
        }
      }
      mled.display();
      delay(customSpeed);
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot((y+1)-z-x,4+y,0); // clean dot
          mled.dot((y+1)-z-x,4-y,0); // clean dot
        }
      }
    }
}

void turnLeftNEW (int customIntensity, int customSpeed) {
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

void turnRightNEW (int customIntensity, int customSpeed) {
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

void turnRight (int customIntensity, int customSpeed) {
    mled.intensity=customIntensity;//change intensity
    for (int z=0;z<8;z++){
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot((3-y)+z+x,4+y); // clean dot
          mled.dot((3-y)+z+x,4-y); // clean dot
        }
      }
      mled.display();
      delay(customSpeed);
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot((3-y)+z+x,4+y,0); // clean dot
          mled.dot((3-y)+z+x,4-y,0); // clean dot
        }
      }
    }
}

void turnUp (int customIntensity, int customSpeed) {
    mled.intensity=customIntensity;//change intensity
    for (int x=0;x<8;x++) {
     for (int y=0;y<8;y++) {
        mled.dot(x,y,0); // clean dot
      }
    }
    for (int z=0;z<8;z++){
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot(4+y,(3-y)+z+x); 
          mled.dot(4-y,(3-y)+z+x);
        }
      }
      mled.display();
      delay(customSpeed);
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot(4+y,(3-y)+z+x,0); // clean dot
          mled.dot(4-y,(3-y)+z+x,0); // clean dot
        }
      }
    }
    mled.display();
}

void turnDown (int customIntensity, int customSpeed) {
    mled.intensity=customIntensity;//change intensity
    for (int z=0;z<8;z++){
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot(4+y,(y+1)-z-x); // clean dot
          mled.dot(4-y,(y+1)-z-x); // clean dot
        }
      }
      mled.display();
      delay(customSpeed);
      for (int x=0;x<2;x++) {
        for (int y=3;y>=0;y--) {
          mled.dot(4+y,(y+1)-z-x,0); // clean dot
          mled.dot(4-y,(y+1)-z-x,0); // clean dot
        }
      }
    }
}

void activeBrake (int customIntensity) {
    mled.intensity=customIntensity;//change intensity
      for (int x=0;x<8;x++) {
        for (int y=0;y<8;y++) {
          mled.dot(x,y); // clean dot
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
    switchMode = 1;
    deActiveBrake();
  }
  if (xPosition >= 640 and buttonRelease) {
    switchMode = 2; 
    deActiveBrake();
  } 
  if (buttonStateJoy == 0) {
    switchMode = 0; 
  } 

  xPosition = analogRead(xPin);
  float checkPosition = previousXValue - xPosition;
  Serial.println(abs(checkPosition));
  Serial.println((checkPosition));
  if (abs(checkPosition) > 150) {
    buttonRelease = true;
  } else {
    buttonRelease = false;
  }
  
  previousXValue = xPosition;
  buttonStateJoy = digitalRead(buttonPin);
}

void loop() {
  if (switchMode ==1) {
    turnRightNEW(5,60);
  } else if (switchMode ==2) {
    turnLeftNEW(5, 60);
  } else if (switchMode ==0) { 
    deActiveBrake();
  }
  delay(50);
  readingJoyStick ();
}
