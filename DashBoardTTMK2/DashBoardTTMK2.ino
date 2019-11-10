#include <Wire.h>
#include <OBD2UART.h>
#include<Arduino.h>
#include<U8g2lib.h>



COBD obd;

#ifdef U8X8_HAVE_HW_SPI
#include<SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include<Wire.h>
#endif

//Set parameters for the screen
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//Define button and LED NANO
//const int  buttonPin = 2;    // the pin that the pushbutton is attached to
//const int ledPin = 12;       // the pin that the LED is attached to

//Define button and LED STM32
const int  buttonPin = PB3;    // the pin that the pushbutton is attached to
const int ledPin = PB1;       // the pin that the LED is attached to

// Define vars for screen rotation
int currentScreen = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

//Define the screen pages
int number_of_screens = 3;

//Define common Gauge parameters
byte radius=23; //radius
byte percent=120; //needle percent
int n=(radius/100.00)*percent;
//Define parameters for left Gauge
byte Leftcenterx=32; //x center
byte Leftcentery=36; //y center
float Leftgs_rad=1.572;
float Leftge_rad=4.887;
//Define parameters for the Right Gauge
byte Rightcenterx=97; //x center
byte Rightcentery=36; //y center
float Rightgs_rad=4.712;
float Rightge_rad=1.396;


// Define the bitmap
#define Audi_splash2_width 128
#define Audi_splash2_height 64
static const unsigned char Audi_splash2_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x1f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf8, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe,
   0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x7f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xfc, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x1f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf8, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0,
   0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xf0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x7f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xfe, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0,
   0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x07, 0xc0, 0x7f, 0x00,
   0xf0, 0x0f, 0x80, 0xff, 0xff, 0x01, 0xfc, 0x03, 0x00, 0x00, 0xff, 0xff,
   0x0f, 0xc0, 0x7f, 0x00, 0xf8, 0x0f, 0xe0, 0xff, 0xff, 0x03, 0xfc, 0x07,
   0x00, 0x80, 0xff, 0xff, 0x0f, 0xc0, 0x7f, 0x00, 0xf8, 0x0f, 0xf8, 0xff,
   0xff, 0x07, 0xfc, 0x07, 0x00, 0xc0, 0xff, 0xf9, 0x1f, 0xc0, 0x7f, 0x00,
   0xf8, 0x0f, 0xfe, 0xff, 0xff, 0x07, 0xfc, 0x07, 0x00, 0xc0, 0xff, 0xf9,
   0x3f, 0xc0, 0x7f, 0x00, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0x0f, 0xfc, 0x07,
   0x00, 0xe0, 0xff, 0xf0, 0x3f, 0xc0, 0x7f, 0x00, 0xf8, 0x8f, 0xff, 0xff,
   0xff, 0x1f, 0xfc, 0x07, 0x00, 0xf0, 0x7f, 0xe0, 0x7f, 0xc0, 0x7f, 0x00,
   0xf8, 0xcf, 0xff, 0x03, 0xfc, 0x3f, 0xfc, 0x07, 0x00, 0xf0, 0x7f, 0xe0,
   0xff, 0xc0, 0x7f, 0x00, 0xf8, 0xcf, 0xff, 0x01, 0xf0, 0x3f, 0xfc, 0x07,
   0x00, 0xf8, 0x3f, 0xc0, 0xff, 0xc0, 0x7f, 0x00, 0xf8, 0xcf, 0x7f, 0x00,
   0xe0, 0x7f, 0xfc, 0x07, 0x00, 0xfc, 0x1f, 0x80, 0xff, 0xc1, 0x7f, 0x00,
   0xf8, 0xef, 0x7f, 0x00, 0xc0, 0x7f, 0xfc, 0x07, 0x00, 0xfe, 0x0f, 0x80,
   0xff, 0xc3, 0x7f, 0x00, 0xf8, 0xef, 0x3f, 0x00, 0xc0, 0x7f, 0xfc, 0x07,
   0x00, 0xfe, 0x0f, 0x00, 0xff, 0xc3, 0x7f, 0x00, 0xf8, 0xef, 0x3f, 0x00,
   0xc0, 0x7f, 0xfc, 0x07, 0x00, 0xff, 0xff, 0xff, 0xff, 0xc7, 0x7f, 0x00,
   0xf8, 0xef, 0x7f, 0x00, 0xc0, 0x7f, 0xfc, 0x07, 0x80, 0xff, 0xff, 0xff,
   0xff, 0xcf, 0xff, 0x00, 0xfc, 0xcf, 0x7f, 0x00, 0xe0, 0x7f, 0xfc, 0x07,
   0x80, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0x01, 0xfe, 0xcf, 0xff, 0x00,
   0xf0, 0x7f, 0xfc, 0x07, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0x07,
   0xff, 0xcf, 0xff, 0x03, 0xf8, 0x3f, 0xfc, 0x07, 0xe0, 0xff, 0xff, 0xff,
   0xff, 0xbf, 0xff, 0xff, 0xff, 0x87, 0xff, 0xff, 0xff, 0x1f, 0xfc, 0x07,
   0xe0, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x07, 0xff, 0xff,
   0xff, 0x0f, 0xfc, 0x07, 0xf0, 0x7f, 0x00, 0x00, 0xf0, 0x7f, 0xfe, 0xff,
   0xff, 0x03, 0xfe, 0xff, 0xff, 0x07, 0xfc, 0x07, 0xf8, 0x3f, 0x00, 0x00,
   0xe0, 0xff, 0xfc, 0xff, 0xff, 0x00, 0xfc, 0xff, 0xff, 0x03, 0xfc, 0x07,
   0xfc, 0x1f, 0x00, 0x00, 0xc0, 0xff, 0xf0, 0xff, 0x7f, 0x00, 0xf0, 0xff,
   0xff, 0x00, 0xfc, 0x07, 0xfc, 0x1f, 0x00, 0x00, 0xc0, 0xff, 0xc1, 0xff,
   0x0f, 0x00, 0x80, 0xff, 0x3f, 0x00, 0xfc, 0x07, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0x0f, 0xfc, 0xff, 0xff, 0xff, 0x07,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0x8f, 0xff,
   0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
   0xff, 0xff, 0xcf, 0xff, 0xff, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xff, 0x01,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xff,
   0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff,
   0xff, 0xff, 0xe3, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xe1, 0xff, 0xff, 0xff, 0x7f, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x1f, 0x00, 0x00, 0x00,
   0xfc, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0,
   0x1f, 0x00, 0x00, 0x00, 0xfc, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xf8, 0x0f, 0x00, 0x00, 0x00, 0xfc, 0x07, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x0f, 0x00, 0x00, 0x00,
   0xfe, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
   0x07, 0x00, 0x00, 0x00, 0xfe, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xfc, 0x07, 0x00, 0x00, 0x00, 0xff, 0x01, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x07, 0x00, 0x00, 0x00,
   0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe,
   0x03, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xfe, 0x03, 0x00, 0x00, 0x80, 0xff, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x01, 0x00, 0x00, 0x80,
   0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
   0x01, 0x00, 0x00, 0x80, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xc0,
   0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff,
   0x00, 0x00, 0x00, 0xc0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x80, 0x7f, 0x00, 0x00, 0x00, 0xe0, 0x1f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0x00, 0xe0,
   0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x3f,
   0x00, 0x00, 0x00, 0xf0, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xc0, 0x3f, 0x00, 0x00, 0x00, 0xf0, 0x0f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1f, 0x00, 0x00, 0x00, 0xf0,
   0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1f,
   0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf0, 0x0f, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x00, 0x00, 0x00, 0xf8,
   0x03, 0x00, 0x00, 0x00 };

   //define screen1 bitmap
#define gaugebmp_width 128
#define gaugebmp_height 64
static unsigned char gaugebmp[] = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0x0f,
   0xf0, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
   0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08,
   0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
   0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00,
   0x01, 0x00, 0x08, 0x00, 0x00, 0xe8, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08,
   0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x17, 0x00, 0x00, 0x08, 0x03, 0x80,
   0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x10, 0x00,
   0x00, 0x08, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00,
   0x01, 0x00, 0x10, 0x00, 0x00, 0x04, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08,
   0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x20, 0x00, 0x00, 0x04, 0x00, 0x80,
   0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x20, 0x00,
   0x00, 0x04, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00,
   0x01, 0x00, 0x20, 0x00, 0x00, 0x04, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08,
   0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x20, 0x00, 0x00, 0x02, 0x00, 0x80,
   0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00,
   0x00, 0x02, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00,
   0x01, 0x00, 0x40, 0x00, 0x00, 0x02, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08,
   0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00, 0x00, 0x02, 0x00, 0x80,
   0xff, 0xff, 0xff, 0x0f, 0xf0, 0xff, 0xff, 0xff, 0x01, 0x00, 0x40, 0x00,
   0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x04, 0x00, 0x00,
   0x00, 0x00, 0x40, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00,
   0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x02, 0x00, 0x00,
   0x00, 0x00, 0x20, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00,
   0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x00,
   0x00, 0x00, 0x20, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
   0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x04, 0x00, 0x00,
   0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00,
   0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00,
   0x00, 0x00, 0x20, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
   0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x00,
   0x00, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
   0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x20, 0x00, 0x00,
   0x00, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
   0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x20, 0x00, 0x00,
   0x00, 0x00, 0x02, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00,
   0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00,
   0x00, 0x00, 0x02, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
   0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x80, 0x00, 0x00,
   0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00,
   0x00, 0x00, 0x03, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00,
   0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x10, 0x00, 0x00,
   0x00, 0x00, 0x08, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
   0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x10, 0x00, 0x00,
   0x00, 0x00, 0x70, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00,
   0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0xc0, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
   0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xe0, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0,
   0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00 };

//Function to draw the bitmap
void drawSplash() {
 // graphic commands to redraw the complete screen should be placed here
  u8g2.drawXBMP( 0, 0, Audi_splash2_width, Audi_splash2_height, Audi_splash2_bits);
}

//Function to draw the bitmap
void drawGauge() {
 // graphic commands to redraw the complete screen should be placed here
  u8g2.drawXBMP( 0, 0, gaugebmp_width, gaugebmp_height , gaugebmp);
}

//Funtion reconnect when no OBD connection
void reconnect()
{
  //Turn on Led
  digitalWrite(ledPin, HIGH);
  //Print "Reconnecting"
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_roentgen_nbp_t_all);
  u8g2.drawStr(0,15,"Reconnecting");
  u8g2.sendBuffer();
  delay(1000);
  //Loop for OBD INIT to become true
  for (uint16_t i = 0; !obd.init(); i++) {
    if (i == 5) {
      //Turn off Led
      digitalWrite(ledPin, LOW);
      //Clear Screen
      u8g2.clear();
    }
  delay(3000);
  }
}

//function to check SRAM
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

/*Text block for vars
u8g2.setFont(u8g2_font_roentgen_nbp_tr);
u8g2.setCursor(0,50);
u8g2.print(value);*/


//Draw the screen with live values
void DrawScreen(int thescreen) {
  switch (thescreen) {
    case 0: {
        static byte pidsScreen1[2] = {PID_RPM, PID_SPEED};
        int valuesScreen1[sizeof(pidsScreen1)];
        //Draw the bitmap
        drawGauge();
        //parameters for Left Gauge on screen #1
        int LeftminValSCR1 = 0;
        int LeftmaxValSCR1 = 8000;
        //parameters for Left Gauge on screen #1
        int RightminValSCR1 = 0;
        int RightmaxValSCR1 = 240;
        if(obd.readPID(pidsScreen1, sizeof(pidsScreen1), valuesScreen1) == sizeof(pidsScreen1)) {
          // Draw Left Gauge
          float LeftiSCR1 = ((valuesScreen1[0] - 0) * (Leftge_rad - Leftgs_rad) / (LeftmaxValSCR1 - LeftminValSCR1) + Leftgs_rad);
          int LeftxpSCR1 = Leftcenterx + (sin(LeftiSCR1) * n);
          int LeftypSCR1 = Leftcentery - (cos(LeftiSCR1) * n);
          u8g2.drawLine(Leftcenterx, Leftcentery, LeftxpSCR1, LeftypSCR1);
          //Draw Right Gauge
          float RightiSCR1 = ((valuesScreen1[1] - 0) * (Rightge_rad - Rightgs_rad) / (RightmaxValSCR1 - RightminValSCR1) + Rightgs_rad);
          int RightxpSCR1 = Rightcenterx + (sin(RightiSCR1) * n);
          int RightypSCR1 = Rightcentery - (cos(RightiSCR1) * n);
          u8g2.drawLine(Rightcenterx, Rightcentery, RightxpSCR1, RightypSCR1);
          //Print values and text on screen
          u8g2.setFont(u8g2_font_profont12_mf);
          u8g2.drawStr(4, 14, "RPM");
          u8g2.setCursor(35, 31);
          u8g2.print((unsigned int)valuesScreen1[0] % 10000);
          u8g2.drawStr(69, 14, "SPEED");
          u8g2.setCursor(74, 31);
          u8g2.print((unsigned int)valuesScreen1[1] % 1000);
          // light on LED when RPM exceeds 4000
          digitalWrite(ledPin, valuesScreen1[0] > 4000 ? HIGH : LOW);
        }
        else {
          u8g2.setFont(u8g2_font_profont15_mf);
          u8g2.drawStr(64,32, "ERROR");
        }
      break;
    }
    case 1: {
      static byte pidsScreen2[2] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP};
      int valuesScreen2[sizeof(pidsScreen2)];
      //Draw the bitmap
      drawGauge();
      //parameters for Left Gauge on screen #1
      int LeftminValSCR2 = -20;
      int LeftmaxValSCR2 = 120;
      //parameters for Left Gauge on screen #1
      int RightminValSCR2 = -20;
      int RightmaxValSCR2 = 60;
        // we weten welke pids we gaan ophalen
        if(obd.readPID(pidsScreen2, sizeof(pidsScreen2), valuesScreen2) == sizeof(pidsScreen2)) {
          // Draw Left Gauge
          float LeftiSCR2 = ((valuesScreen2[0] - 0) * (Leftge_rad - Leftgs_rad) / (LeftmaxValSCR2 - LeftminValSCR2) + Leftgs_rad);
          int LeftxpSCR2 = Leftcenterx + (sin(LeftiSCR2) * n);
          int LeftypSCR2 = Leftcentery - (cos(LeftiSCR2) * n);
          u8g2.drawLine(Leftcenterx, Leftcentery, LeftxpSCR2, LeftypSCR2);
          //Draw Right Gauge
          float RightiSCR2 = ((valuesScreen2[1] - 0) * (Rightge_rad - Rightgs_rad) / (RightmaxValSCR2 - RightminValSCR2) + Rightgs_rad);
          int RightxpSCR2 = Rightcenterx + (sin(RightiSCR2) * n);
          int RightypSCR2 = Rightcentery - (cos(RightiSCR2) * n);
          u8g2.drawLine(Rightcenterx, Rightcentery, RightxpSCR2, RightypSCR2);
          //Print values and text on screen
          u8g2.setFont(u8g2_font_profont12_mf);
          u8g2.drawStr(4, 14, "COOLANT");
          u8g2.setCursor(37, 31);
          u8g2.print(valuesScreen2[0]);
          u8g2.drawStr(69, 14, "INTAKE");
          u8g2.setCursor(74, 31);
          u8g2.print(valuesScreen2[1]);
        }
        else {
          u8g2.setFont(u8g2_font_profont15_mf);
          u8g2.drawStr(64,32, "ERROR");
        }
      break;
    }
    case 2: {
      byte pidsScreen3[2] = {PID_BAROMETRIC,PID_CATALYST_TEMP_B1S1};
      int valuesScreen3[sizeof(pidsScreen3)];
      //Draw the bitmap
      drawGauge();
      //parameters for Left Gauge on screen #1
      int LeftminValSCR3 = 0;
      int LeftmaxValSCR3 = 800;
      //parameters for Left Gauge on screen #1
      int RightminValSCR3 = 0;
      int RightmaxValSCR3 = 100;
        // we weten welke pids we gaan ophalen
        if(obd.readPID(pidsScreen3, sizeof(pidsScreen3), valuesScreen3) == sizeof(pidsScreen3)) {
          // Draw Left Gauge
          float LeftiSCR3 = ((valuesScreen3[0] - 0) * (Leftge_rad - Leftgs_rad) / (LeftmaxValSCR3 - LeftminValSCR3) + Leftgs_rad);
          int LeftxpSCR3 = Leftcenterx + (sin(LeftiSCR3) * n);
          int LeftypSCR3 = Leftcentery - (cos(LeftiSCR3) * n);
          u8g2.drawLine(Leftcenterx, Leftcentery, LeftxpSCR3, LeftypSCR3);
          //Draw Right Gauge
          float RightiSCR3 = ((valuesScreen3[1] - 0) * (Rightge_rad - Rightgs_rad) / (RightmaxValSCR3 - RightminValSCR3) + Rightgs_rad);
          int RightxpSCR3 = Rightcenterx + (sin(RightiSCR3) * n);
          int RightypSCR3 = Rightcentery - (cos(RightiSCR3) * n);
          u8g2.drawLine(Rightcenterx, Rightcentery, RightxpSCR3, RightypSCR3);
          //Print values and text on screen
          u8g2.setFont(u8g2_font_profont12_mf);
          u8g2.drawStr(4, 14, "BAR");
          u8g2.setCursor(37, 31);
          u8g2.print(valuesScreen3[0]);
          u8g2.drawStr(69, 14, "CAT TEMP");
          u8g2.setCursor(74, 31);
          u8g2.print(valuesScreen3[1]);
        }
        else{
          u8g2.setFont(u8g2_font_profont15_mf);
          u8g2.drawStr(64,32, "ERROR");
        }
      break;
    }
  }
}

void setup()
{
  //Init ledPin as output
  pinMode(ledPin, OUTPUT);
  //Init Screen
  u8g2.begin();
  delay(200);
   //Init OBD UART
  obd.begin();
  //Turn on Led
  digitalWrite(ledPin, HIGH);
  // Drawing the splash screen
  u8g2.clearBuffer();
  drawSplash();
  u8g2.sendBuffer();
  delay(2000);
 //Connect to OBD
  while (!obd.init());
  //Turn of ledPin
  digitalWrite(ledPin, LOW);
  //Clear screen and go to loop
  u8g2.clear();
}

void loop()
{


  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      currentScreen++;
      currentScreen = currentScreen % number_of_screens;
      // Delay a little bit to avoid bouncing
      delay(50);
      u8g2.clear();
    }
  }
// save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;

  //Draw the screens
  u8g2.clearBuffer();
  DrawScreen(currentScreen);
  u8g2.sendBuffer();

//Reconnect if no connection
  if (obd.errors >= 2) {
      delay(2000);
      reconnect();
      setup();
  }
}
