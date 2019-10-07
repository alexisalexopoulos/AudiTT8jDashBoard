


/*************************************************************************
* Simple OBD Data Display
* Works with any Arduino board connected with SH1106 128*64 I2C OLED and
* Freematics OBD-II UART Adapter - https://freematics.com/products
* Distributed under public domain
* Written by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Wire.h>
#include <OBD2UART.h>
#include <MicroLCD.h>
#include<Arduino.h>
#include<U8g2lib.h>
#include <U8x8lib.h>



LCD_SH1106 lcd;
COBD obd;

#ifdef U8X8_HAVE_HW_SPI
#include<SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include<Wire.h>
#endif

//Set parameters for the screen 
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  //constructor for direct display
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  //constructor for buffer display

//Define button and LED
const int  buttonPin = 2;    // the pin that the pushbutton is attached to
const int ledPin = 12;       // the pin that the LED is attached to

// Define vars for screen rotation
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

//Defining array for PIDS
static byte pids[]= {PID_RPM, PID_SPEED, PID_INTAKE_TEMP, PID_COOLANT_TEMP};
//assigning an index to the array
static byte index = 0;
int value;
 
//Define the screen pages
int number_of_screens = 4; 
 
// Define the bitmap
#define download_width 128
#define download_height 64
static const unsigned char download_bits[] PROGMEM = {
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

//Function to draw the bitmap
void drawSplash(void) {
 // graphic commands to redraw the complete screen should be placed here  
 u8g2.drawXBMP( 0, 0, download_width, download_height, download_bits);
}

//Funtion reconnect when no OBD connection
void reconnect()
{
  u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_roentgen_nbp_t_all);
      u8g2.drawStr(0,15,"Reconnecting");
    } while ( u8g2.nextPage() );
      delay(1000);
  //digitalWrite(SD_CS_PIN, LOW);
  for (uint16_t i = 0; !obd.init(); i++) {
    if (i == 5) {
      u8g2.clear();
    }
    delay(3000);
  }
}

//Function to retreive and display the data
void showData(byte pid, int value)
{
  switch (pid) {                                
    case PID_COOLANT_TEMP:
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_chroma48medium8_8u);
        u8g2.drawStr(0,10,"Coolant temp");
        u8g2.drawStr(30,10,value);
      } while ( u8g2.nextPage() );
      delay(1000);
    break;
  case PID_INTAKE_TEMP:
      if (value >= 0 && value < 100) {
        u8g2.firstPage();
          do {
            u8g2.setFont(u8g2_font_chroma48medium8_8u);
            u8g2.drawStr(0,20,"Intake temp");
            u8g2.drawStr(30,20,value);
          } while ( u8g2.nextPage() );
          delay(1000);  
      }
    break;
  case PID_SPEED:
     u8g2.firstPage();
          do {
            u8g2.setFont(u8g2_font_chroma48medium8_8u);
            u8g2.drawStr(0,40,"Intake temp");
            u8g2.setCursor(30, 40);
            u8g2.print((unsigned int)value % 1000, 3);
          } while ( u8g2.nextPage() );
          delay(1000); 
      break;
  case PID_RPM:
  u8g2.firstPage();
          do {
            u8g2.setFont(u8g2_font_chroma48medium8_8u);
            u8g2.drawStr(0,50,"Intake temp");
            u8g2.setCursor(30, 50);
            u8g2.print((unsigned int)value % 10000, 4);
          } while ( u8g2.nextPage() );
          delay(1000); 
    break;
  }
}



//Code to write static data to the screen during setup
void initScreen()
{
  u8g2.setFont(u8g2_font_roentgen_nbp_t_all);
  delay(1000);
 
}

void setup()
{
    // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);
  //Initializing the screen
  u8g2.begin();;
  delay(100);
  // Drawing the splash screen
  drawSplash();
  delay(5000);
  

}

void loop()
{
  byte pid = pids[index];
  // send a query to OBD adapter for specified OBD-II pid
  if (obd.readPID(pid, value)) {

     // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      u8g2.clear();
      delay(50);
      // if the current state is HIGH then the button went from off to on:
      buttonPushCounter++;
      buttonPushCounter = buttonPushCounter % number_of_screens;
      //0 = PID_SPEED
      //1 = PID_RPM
      //2 = PID_INTAKE_TEMP
      //3 = PID_COOLANT_TEMP
        switch (buttonPushCounter)
        {
        case 1:
          showData(pid, value);
          break;
        case 2:
          showData(pid, value);
          break;
        case 3:
          showData(pid, value);
          break;
        case 4:
          showData(pid, value);
          break;
        }
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
}
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
  index = (index + 1) % sizeof(pids);


  if (obd.errors >= 2) {
      delay(5000);
      reconnect();
      setup(); 
  } 
}
