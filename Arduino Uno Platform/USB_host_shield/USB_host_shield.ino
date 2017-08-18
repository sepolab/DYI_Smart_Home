/* USB related */
//#include <Spi.h>
#include <Usb.h>
#include <CP210x.h>
//#include <max3421e.h>
//#include <Max3421e_constants.h>



#include "board_test.h" /* Board test messages */

#define CP2103_VendID               0x10C4
#define CP2103_ProdID               0xEA60
#define CP2103_RevID                0x0100
#define REQTYPE_HOST_TO_DEVICE      0x40
#define REQTYPE_HOST_TO_INTERFACE   0x41
#define REQTYPE_DEVICE_TO_HOST      0xc0

#define CP210X_IFC_ENABLE           0x00
#define CP210X_GET_BAUDRATE         0x1D
#define CP210X_SET_BAUDRATE         0x1E
#define CP210X_SET_FLOW             0x13
#define CP210X_GET_FLOW             0x14
#define USB_CTRL_SET_TIMEOUT        5000
#define UART_ENABLE                 0x0001

typedef void (*PARSE)( uint8_t bytes );

//#define MAX_SS 10

void setup();
void loop();

MAX3421E Max;
USB Usb;

byte result;
byte tmp;
char data[16];
byte rcv_bytes;


void setup()
{
  Serial.begin( 115200 );
  Max.powerOn();
  printProgStr( startBanner );
  printProgStr( anykey_msg );
  //Serial.print( Max.getvar(), DEC);
}

void loop()
{
  DEV_RECORD devtable[ USB_NUMDEVICES + 1 ];
  /* start tests */
  Serial.println("revregcheck...");
  if (!revregcheck()) test_halted();

  Serial.println("now osc test...");
  if (!osctest()) printProgStr(PSTR("OSCOK test failed. Check the oscillator"));
  if (!usbtest()) printProgStr(PSTR("USB connection test failed. Check traces from USB connector to MAX3421E, as well as VBUS"));  //never gets here
    /* All tests passed */
  Serial.println("\nLink is ready.\n\n");

  Serial.println("Calling Usb.task()...");
  Usb.Task();
  
  // unsure if we need to first "open" the CP210x...
  Serial.println("\nAttempting to open device...");
  //clearData();
  result = Usb.ctrlReq( 1, 0, REQTYPE_HOST_TO_DEVICE, CP210X_IFC_ENABLE, UART_ENABLE, 0x00, 0x0000, 0x00, NULL, USB_NAK_LIMIT);
  if(result) {
    Serial.print("Error: result of command is: 0x");
    Serial.println(result);
  }
  //printData();


  Serial.println("\nRead initial baud rate setting...");
  clearData();
  result = Usb.ctrlReq( 1, 0, REQTYPE_DEVICE_TO_HOST, CP210X_GET_BAUDRATE, 0x00, 0x00, 0x0000, 0x08, data, USB_NAK_LIMIT);
  if(result) {
    Serial.print("Error: result of command is: 0x");
    Serial.println(result);
  }
  printData();
  
  Serial.println("\nAttempting to set baud rate to 57600");
  result = Usb.ctrlReq( 1, 0, REQTYPE_HOST_TO_DEVICE, CP210X_SET_BAUDRATE, 0xE1, 0x00, 0x0000, 0x00, NULL, USB_NAK_LIMIT);
  if(result) {
    Serial.print("Error: result of command is: 0x");
    Serial.println(result);
  }
  else Serial.println("Success.");
  
  Serial.println("Read back baud rate...");
  clearData();
  result = Usb.ctrlReq( 1, 0, REQTYPE_DEVICE_TO_HOST, CP210X_GET_BAUDRATE, 0x00, 0x00, 0x0000, 0x04, data, USB_NAK_LIMIT);
  if(result) {
    Serial.print("Error: result of command is: 0x");
    Serial.println(result);
  }
  printData();

  Serial.println("\nAttempting to set flow control...");
  result = Usb.ctrlReq( 1, 0, REQTYPE_HOST_TO_DEVICE, CP210X_SET_FLOW, 0x41, 0x00, 0x00, 0x00, NULL, USB_NAK_LIMIT);
  if(result) {
    Serial.print("Error: result of command is: 0x");
    Serial.println(result);
  }

  Serial.println("Read back flow control settings...");
  clearData();
  result = Usb.ctrlReq( 1, 0, REQTYPE_DEVICE_TO_HOST, CP210X_GET_FLOW, 0x00, 0x00, 0x0000, 0x10, data, USB_NAK_LIMIT);
  if(result) {
    Serial.print("Error: result of command is: 0x");
    Serial.println(result);
  }
  printLongData();

  Serial.println("\nAttempting to set baud rate to 115200:");
  result = Usb.ctrlReq( 1, 0, REQTYPE_HOST_TO_DEVICE, CP210X_SET_BAUDRATE, 0xC2, 0x01, 0x00, 0x00, NULL, USB_NAK_LIMIT);
  if(result) {
    Serial.print("Error: result of command is: 0x");
    Serial.println(result);
  }

  Serial.println("Now read back baud rate again to confirm change...");
  clearData();
  result = Usb.ctrlReq( 1, 0, REQTYPE_DEVICE_TO_HOST, CP210X_GET_BAUDRATE, 0x00, 0x00, 0x0000, 0x04, data, USB_NAK_LIMIT);
  if(result) {
    Serial.print("Error: result of command is: 0x");
    Serial.println(result);
  }
  printData();

  while(1); // stop here for now....
  
  // issue transfer_IN commands at 8 bytes at a time... forever...
  for(byte i=0; i<2; i++) {
    Serial.println("\n\nAttempting to receive data...");

    //byte USB::inTransfer( addr, ep, unsigned int nbytes, char* data, unsigned int nak_limit )
    result = BCinTransfer( 1, 0, 8, data, USB_NAK_LIMIT);

    Serial.print("Result of inTransfer command: 0x0");
    Serial.println(result, HEX);
        
    if(result == 0) {
      for(byte i=0; i<8; i++) {
         Serial.print(data[i]);
      }
      Serial.println(" ");
    }
    delay(1000);
  }
  while(1); // stop here for now....
}
