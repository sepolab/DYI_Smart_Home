#include <WEMOS_Matrix_LED.h>

MLED mled(0); //set intensity=0
int buzzer = D8; //Buzzer control port, default D8
unsigned long previousMillis1 = 0; //set time for interval publish

const int changeModePIN = D3;
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 500;    // the debounce time; increase if the output flickers
int switchMode = 0;
int buzzerPlay = 0;

int xPin = D3;
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
    for (int x=0;x<8;x++) {
     for (int y=0;y<8;y++) {
        mled.dot(x,y,0); // clean dot
      }
    }
     mled.display();
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
    for (int x=0;x<8;x++) {
     for (int y=0;y<8;y++) {
        mled.dot(x,y,0); // clean dot
      }
    }
     mled.display();
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
    for (int x=0;x<8;x++) {
     for (int y=0;y<8;y++) {
        mled.dot(x,y,0); // clean dot
      }
    }
     mled.display();
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

void deActiveBrake (int customIntensity) {
    mled.intensity=customIntensity;//change intensity
      for (int x=0;x<8;x++) {
        for (int y=0;y<8;y++) {
          mled.dot(x,y,0); // clean dot
        }
      }
      mled.display();
}

void loop() {
  if (xPosition == 0) {
    switchMode = 1;
  }
  if (yPosition == 0) {
    switchMode = 2; 
  } 
  if (buttonStateJoy == 0) {
    switchMode = 0; 
  } 

  if (switchMode ==1) {
    turnRight(4,60);
  } else if (switchMode ==2) {
    turnLeft(4,60);
  } 
  
  if (switchMode ==1) {
    turnRight(4,60);
  } else if (switchMode ==2) {
    turnLeft(4,60);
  } else if (switchMode ==0) { 
    deActiveBrake(7);
  }
  
  xPosition = digitalRead(xPin);  
  yPosition = digitalRead(yPin);
  buttonStateJoy = digitalRead(buttonPin);
  Serial.print("X: ");
  Serial.print(xPosition);
  Serial.print(" | Y: ");
  Serial.print(yPosition);
  Serial.print(" | Button: ");
  Serial.println(buttonStateJoy);
}
