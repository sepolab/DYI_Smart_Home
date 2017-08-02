#include <Wire.h>
#include <Adafruit_GFX.h>  //https://github.com/mcauser/Adafruit_SSD1306/tree/esp8266-64x48
#include <Adafruit_SSD1306.h> //https://github.com/mcauser/Adafruit_SSD1306/tree/esp8266-64x48
#include <WEMOS_SHT3X.h> //https://github.com/wemos/WEMOS_SHT3x_Arduino_Library

#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);

SHT3X sht30(0x45);

void setup() {

  Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

}

void loop() {
  
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(WHITE);

  if(sht30.get()==0){
    display.println("Nhiet Do: ");
    Serial.println("T: ");
    display.setTextSize(2);
    display.print(int(sht30.cTemp));
    display.setTextSize(1);
    display.print("*C");
    Serial.println(sht30.cTemp);
    display.setTextSize(2);
    display.println();
    display.setTextSize(1);
    display.println("Do Am: ");
    Serial.println("H: ");
    display.setTextSize(2);
    display.print(int(sht30.humidity));
    Serial.println(sht30.humidity);
    display.setTextSize(1);
    display.print("%");
  }
  else
  {
    display.println("Error!");
    Serial.println("Error");
  }
  display.display();
  delay(1000);

}
