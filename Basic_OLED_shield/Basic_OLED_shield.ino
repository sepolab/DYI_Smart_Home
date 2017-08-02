/**
  SePoLab SMART Devices PROJECT
  Name: Basic_OLED_Shield
  
  Purpose: calculate temperature + humidity then display to micro OLEDscreen 64x48 pixels
  
  General Hardware:
  - BASE: 
  -- WeMosD1mini/WeMosD1miniPro(required if supported upgrade firmware via OTA) / WeMosD1Lite (no OTA support)
  - SHIELD:
  -- DYI 2switches shield v1.1.0(required) - used Port: D1;D2;RST;D3
  -- 1-BUTTON shield v.2.0.0(optional) - used Port: D3
  -- OLED Shield v1.1.0 (required) - used Port: D1;D2 for I2C
  -- SHT30 Shield v1.0.0 (required) - used Port: D1;D2 for I2C
  
  - CONNECTION:
  -- SOFT RESET BUTTON: D3 => use to reset WiFi or MQTT server connection
  -- OLED Shield uses D1;D2
  -- SHT30 shield uses D1; D2
  
Pin Function                          ESP-8266 Pin
TX  TXD                                     TXD
RX  RXD                                     RXD
A0  Analog input                            A0
D0  IO                                      GPIO16
D1  IO, SCL                                 GPIO5
D2  IO, SDA                                 GPIO4
D3  IO, 10k Pull-up                         GPIO0
D4  IO, 10k Pull-up, BUILTIN_LED            GPIO2
D5  IO, SCK                                 GPIO14
D6  IO, MISO                                GPIO12
D7  IO, MOSI                                GPIO13
D8  IO, 10k Pull-down, SS                   GPIO15
G   Ground                                  GND
5V  5V                                      -
3V3 3.3V                                    3.3V
RST Reset                                   RST


  @author Sebastian sepolab@gmail.com
  @version 1.0 6/15/17
*/

#include <Wire.h>
#include <Adafruit_GFX.h>  //https://github.com/mcauser/Adafruit_SSD1306/tree/esp8266-64x48
#include <Adafruit_SSD1306.h> //https://github.com/mcauser/Adafruit_SSD1306/tree/esp8266-64x48
#include <WEMOS_SHT3X.h> //https://github.com/wemos/WEMOS_SHT3x_Arduino_Library

#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);

SHT3X sht30(0x45);
bool switchMode = true;

void setup() {
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
}

void loop() {
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.setTextColor(WHITE);

  if(sht30.get()==0){
    if (switchMode) {
      display.println("TEMPT");
      display.setTextSize(1);
      display.println("  -----");
      display.setTextSize(2);
      Serial.println("T: ");
      display.setTextSize(3);
      display.print(int(sht30.cTemp));
      display.setTextSize(1);
      display.print("*");
      Serial.println(sht30.cTemp);
      display.setTextSize(3);
      display.print("C");
      switchMode = false;
    } else {
      display.println("HUMID");
      display.setTextSize(1);
      display.println("  -----");
      display.setTextSize(2);
      Serial.println("T: ");
      display.setTextSize(3);
      display.print(int(sht30.humidity));
      display.setTextSize(1);
      display.print(" ");
      Serial.println(sht30.cTemp);
      display.setTextSize(3);
      display.print("%");
      switchMode = true;      
    }
  }
  else
  {
    display.println("Error!");
    Serial.println("Error");
  }
  display.display();
  delay(2000);

}
