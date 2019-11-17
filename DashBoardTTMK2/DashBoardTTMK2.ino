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
int number_of_screens = 4;

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


// Define the bitmap for splashscreen
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

   //define boeing gauge bitmap
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

//Function to draw Gforce and Horizontal Bars
#define G_HBars_width 128
#define G_HBars_height 64
static const unsigned char G_HBars_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0x00,
   0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x80, 0xff,
   0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf0, 0x01, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x78, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00,
   0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x80, 0x03, 0x00, 0x01, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00,
   0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
   0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x18, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f,
   0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xfe, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0x7f, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
   0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00,
   0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x07, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00,
   0x01, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x0e, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x78, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03,
   0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xc0, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00 };

#define BarGraph_width 128
#define BarGraph_height 64
static const unsigned char BarGraph_bits [] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00,
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
   0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x06, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x40,
   0x00, 0x00, 0x00, 0x60, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f,
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
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0xfe, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f,
   0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
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
   0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00,
   0x02, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60,
   0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00 };

   void drawBitmap(const uint8_t *bitmap) {
 // graphic commands to redraw the complete screen should be placed here
  u8g2.drawXBMP( 0, 0, 128, 64, bitmap);
}

//Function to draw the bitmap for Gforce speed values
void drawpage1() {
 // graphic commands to redraw the complete screen should be placed here
  u8g2.drawXBMP( 0, 0, G_HBars_width, G_HBars_height, G_HBars_bits);
}

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

//Function to draw progress Bars
void drawProgressbar(int x,int y, int width,int height, int progress, int progressBarMaxValue)
{
  float progressbar = ((float)(width-4) / (progressBarMaxValue)) * progress;
  u8g2.drawBox(x, y, progressbar , height-4 );

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


/*Text block for vars
u8g2.setFont(u8g2_font_roentgen_nbp_tr);
u8g2.setCursor(0,50);
u8g2.print(value);*/


//Draw the screen with live values
void DrawScreen(int thescreen) {
  switch (thescreen) {
    case 0: {
        static byte pidsScreen1[2] = {PID_SPEED,PID_RPM};
        int valuesScreen1[sizeof(pidsScreen1)];
        //Draw page1 bitmap
        drawBitmap(G_HBars_bits);
        if(obd.readPID(pidsScreen1, sizeof(pidsScreen1), valuesScreen1) == sizeof(pidsScreen1)) {
          //print Text
          u8g2.setFont(u8g2_font_profont10_mf);
          u8g2.drawStr(65, 12, "SPEED");
          u8g2.drawStr(65, 30, "RPM");
          u8g2.drawStr(65, 49, "GFORCE");
          //SPEED
          u8g2.setFont(u8g2_font_profont15_mf);
          u8g2.setCursor(103, 14);
          u8g2.print((unsigned int)valuesScreen1[0] % 1000);
          //RPM
          u8g2.setCursor(100, 33);
          u8g2.print((unsigned int)valuesScreen1[1] % 10000);
          //GFORCE
          //u8g2.setCursor(103, 52);
          //u8g2.print((unsigned int)valuesStatic[1] % 10000);
          // light on LED when RPM exceeds 4000
          digitalWrite(ledPin, valuesScreen1[1] > 3000 ? HIGH : LOW);
        }
        else {
          u8g2.setFont(u8g2_font_profont15_mf);
          u8g2.drawStr(64,32, "ERROR");
        }
      break;
    }
    case 1: {
      static byte pidsScreen2[4] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP,PID_AMBIENT_TEMP,PID_ENGINE_LOAD};
      int valuesScreen2[sizeof(pidsScreen2)];
        //Draw bitmap
        drawBitmap(BarGraph_bits);
        // we weten welke pids we gaan ophalen
        if(obd.readPID(pidsScreen2, sizeof(pidsScreen2), valuesScreen2) == sizeof(pidsScreen2)) {
          //Draw progress Bars
          int pbarwidth = 120;
          int pbarheight = 9;
          drawProgressbar(5,15,pbarwidth,pbarheight,valuesScreen2[0], 130);
          drawProgressbar(5,34,pbarwidth,pbarheight,valuesScreen2[1], 100);
          drawProgressbar(5,53,pbarwidth,pbarheight,valuesScreen2[2], 60);
          // Draw text
          u8g2.setFont(u8g2_font_trixel_square_tf); //5 pixel height
          u8g2.drawStr(5, 13, "COOLANT");
          u8g2.drawStr(5, 32, "INTAKE");
          u8g2.drawStr(5, 51, "AMB TMP");
          u8g2.drawStr(65, 7, "LOAD");
          u8g2.setCursor(110, 13);
          u8g2.print(valuesScreen2[0]);             //Coolant temp
          u8g2.setCursor(110, 32);
          u8g2.print(valuesScreen2[1]);             //Intake temp
          u8g2.setCursor(110, 51);
          u8g2.print(valuesScreen2[2]);             //Ambient temp
          u8g2.setCursor(110,7);
          u8g2.print(valuesScreen2[3]);             //Engine load
          //Draw the symbols
          u8g2.drawStr(35, 13, "(c)");
          u8g2.drawStr(35, 32, "(c)");
          u8g2.drawStr(35, 51, "(c)");
          u8g2.drawStr(95, 7, "(%)");
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
      drawBitmap(gaugebmp);
      //parameters for Left Gauge on screen #1
      int LeftminValSCR3 = 0;
      int LeftmaxValSCR3 = 150;
      //parameters for Left Gauge on screen #1
      int RightminValSCR3 = 0;
      int RightmaxValSCR3 = 800;
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
    case 3: {
        byte pidsScreen4[4] = {PID_MAF_FLOW,PID_THROTTLE,PID_BAROMETRIC,PID_WARMS_UPS};
        int valuesScreen4[sizeof(pidsScreen4)];
        //DrawBitMap
        drawBitmap(BarGraph_bits);
        // we weten welke pids we gaan ophalen
        if(obd.readPID(pidsScreen4, sizeof(pidsScreen4), valuesScreen4) == sizeof(pidsScreen4)) {
          //Draw progress Bars
          int pbarwidth = 120;
          int pbarheight = 9;
          drawProgressbar(5,15,pbarwidth,pbarheight,valuesScreen4[0], 200);
          drawProgressbar(5,34,pbarwidth,pbarheight,valuesScreen4[1], 100);
          drawProgressbar(5,53,pbarwidth,pbarheight,valuesScreen4[2], 150);
          // Draw text
          u8g2.setFont(u8g2_font_trixel_square_tf); //5 pixel height
          u8g2.drawStr(5, 13, "MAF");
          u8g2.drawStr(5, 32, "THROTTLE");
          u8g2.drawStr(5, 51, "BAR");
          u8g2.drawStr(65, 7, "DTC KM");
          u8g2.setCursor(110, 13);
          u8g2.print(valuesScreen4[0]);       //MAF sensor
          u8g2.setCursor(110, 32);
          u8g2.print(valuesScreen4[1]);       //Throttle percentage
          u8g2.setCursor(110, 51);
          u8g2.print(valuesScreen4[2]);       //Barrometric pressure
          u8g2.setCursor(110,7);
          u8g2.print((unsigned int)valuesScreen4[3]);     //Number of warmups after dtc clear
          u8g2.drawStr(35, 13, "(gram/%)");
          u8g2.drawStr(35, 32, "(c)");
          u8g2.drawStr(35, 51, "(kPa)");
          u8g2.drawStr(95, 7, "(#)");
        }
        else{
          u8g2.setFont(u8g2_font_profont15_mf);
          u8g2.drawStr(64,32, "ERROR");
        }
      break;
    }
    case 4: {
        byte pidsScreen4[4] = {PID_CATALYST_TEMP_B1S1,PID_CONTROL_MODULE_VOLTAGE,PID_BAROMETRIC,PID_WARMS_UPS};
        int valuesScreen4[sizeof(pidsScreen4)];
        //DrawBitMap
        drawBitmap(BarGraph_bits);
        // we weten welke pids we gaan ophalen
        if(obd.readPID(pidsScreen4, sizeof(pidsScreen4), valuesScreen4) == sizeof(pidsScreen4)) {
          //Draw progress Bars
          int pbarwidth = 120;
          int pbarheight = 9;
          drawProgressbar(5,15,pbarwidth,pbarheight,valuesScreen4[0], 200);
          drawProgressbar(5,34,pbarwidth,pbarheight,valuesScreen4[1], 100);
          drawProgressbar(5,53,pbarwidth,pbarheight,valuesScreen4[2], 150);
          // Draw text
          u8g2.setFont(u8g2_font_trixel_square_tf); //5 pixel height
          u8g2.drawStr(5, 13, "MAF");
          u8g2.drawStr(5, 32, "THROTTLE");
          u8g2.drawStr(5, 51, "BAR");
          u8g2.drawStr(65, 7, "DTC KM");
          u8g2.setCursor(110, 13);
          u8g2.print(valuesScreen4[0]);       //MAF sensor
          u8g2.setCursor(110, 32);
          u8g2.print(valuesScreen4[1]);       //Throttle percentage
          u8g2.setCursor(110, 51);
          u8g2.print(valuesScreen4[2]);       //Barrometric pressure
          u8g2.setCursor(110,7);
          u8g2.print((unsigned int)valuesScreen4[3]);     //Number of warmups after dtc clear
          u8g2.drawStr(35, 13, "(gram/%)");
          u8g2.drawStr(35, 32, "(c)");
          u8g2.drawStr(35, 51, "(kPa)");
          u8g2.drawStr(95, 7, "(#)");
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
  drawBitmap(Audi_splash2_bits);
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
