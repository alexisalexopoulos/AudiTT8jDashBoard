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



LCD_SH1106 lcd;
COBD obd;

#ifdef U8X8_HAVE_HW_SPI
#include<SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include<Wire.h>
#endif

//Set parameters for the screen 
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
 
// Define the bitmap
#define download_width 128
#define download_height 64
static const unsigned char download_bits[] PROGMEM = {
   0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xc0, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f,
   0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xe0, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
   0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x60, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
   0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x30, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c,
   0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x0c, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x06, 0x1c, 0x10, 0x04, 0x41, 0x10, 0x04, 0x01, 0xe0,
   0x80, 0x20, 0x08, 0x82, 0x20, 0x08, 0x00, 0x06, 0x0c, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x20, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x06,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x07, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0e, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
   0xfe, 0xff, 0x3f, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00,
   0x80, 0xff, 0xff, 0x03, 0xfe, 0xff, 0x1f, 0x00, 0x00, 0xf8, 0xff, 0xff,
   0xff, 0xff, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x01, 0x55, 0x55, 0x1d, 0x00,
   0x00, 0xb8, 0xaa, 0xaa, 0xaa, 0xea, 0x01, 0x00, 0xc0, 0xab, 0xaa, 0x01,
   0x00, 0x00, 0x1c, 0x84, 0x10, 0x18, 0x00, 0x00, 0x00, 0xc0, 0x20, 0x84,
   0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x1c, 0x00, 0x00,
   0x00, 0xe0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00,
   0x00, 0x0c, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x0e, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00,
   0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0e, 0x00, 0x00,
   0x00, 0x70, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00,
   0x00, 0x0e, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00,
   0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x06, 0x00, 0x00,
   0x00, 0x30, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
   0x00, 0x07, 0x00, 0x00, 0x00, 0x38, 0x00, 0x04, 0x30, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x03, 0x84, 0x00, 0x03, 0x00, 0x00, 0x00, 0x18, 0x10, 0x20,
   0x38, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00,
   0x00, 0x18, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00,
   0x00, 0x03, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,
   0x00, 0x80, 0x01, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00,
   0x1c, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0x01, 0x00, 0x00,
   0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00,
   0x80, 0x01, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0x10, 0x80, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00,
   0x00, 0x06, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0x00, 0x06, 0x04, 0x00, 0x06, 0x00, 0x00, 0x00,
   0x00, 0xe0, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00,
   0x00, 0x07, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00,
   0x60, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
   0x00, 0x70, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00,
   0x80, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00,
   0x30, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00,
   0x00, 0x30, 0x00, 0x04, 0x30, 0x00, 0x00, 0x00, 0x80, 0x41, 0x00, 0x80,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
   0x80, 0x01, 0x04, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0x08, 0x00,
   0x38, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x80, 0x80, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x18, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,
   0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x10,
   0x18, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x0c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0xe0,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
   0xe0, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x10, 0x00,
   0x0c, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x06, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x60,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00,
   0x30, 0x20, 0x04, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00,
   0x06, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x03, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x30,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0x00,
   0xf8, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff,
   0x03, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00,
   0x80, 0xff, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x1f,
   0x00, 0x00, 0x00, 0x00 };

//Function to draw the bitmap
void draw(void) {
 // graphic commands to redraw the complete screen should be placed here  
 u8g2.drawXBMP( 0, 0, download_width, download_height, download_bits);
}

//Funtion reconnect when no OBD connection
void reconnect()
{
  lcd.clear();
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.print("Reconnecting");
  //digitalWrite(SD_CS_PIN, LOW);
  for (uint16_t i = 0; !obd.init(); i++) {
    if (i == 5) {
      lcd.clear();
    }
    delay(3000);
  }
}

//Function to retreive and display the data
void showData(byte pid, int value)
{
  switch (pid) {
    case PID_COOLANT_TEMP:
    lcd.setCursor(102, 0);
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.printInt(value, 2);
    lcd.print(" C");
    break;
  case PID_INTAKE_TEMP:
      if (value >= 0 && value < 100) {
          lcd.setCursor(102, 1);
          lcd.setFontSize(FONT_SIZE_SMALL);
          lcd.printInt(value, 2); 
          lcd.print(" C");   
      }
    break;
  case PID_DISTANCE:
    lcd.setCursor(102, 2);
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.printInt(value, 2);
    break;
  case PID_SPEED:
    lcd.setCursor(102, 3);
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.printInt((unsigned int)value % 1000, 3);
    break;
  case PID_RPM:
    lcd.setCursor(102, 4);
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.printInt((unsigned int)value % 10000, 4);
    break;
  }
}

//Code to write static data to the screen during setup
void initScreen()
{
  lcd.clear ();
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.setCursor(0, 0);
  lcd.print("COOLANT TEMP");
  lcd.setCursor(0, 1);
  lcd.print("INTAKE TEMP");
  lcd.setCursor(0, 2);
  lcd.print("DISTANCE");
  lcd.setCursor(0, 3);
  lcd.print("SPEED");
  lcd.setCursor(0, 4);
  lcd.print("RPM");
}

void setup()
{
  //lcd.begin();
  //lcd.setFontSize(FONT_SIZE_MEDIUM);
  //lcd.println("OBD DISPLAY");
  //delay(500);
  obd.begin();
  delay(100);
  u8g2.begin();
  delay(100);
  // picture loop
  u8g2.firstPage();
  do {
      draw();
      } while( u8g2.nextPage() );
  while (!obd.init());
  lcd.begin();
  delay (3000);
  initScreen();
}

void loop()
{
  static byte pids[]= {PID_RPM, PID_SPEED, PID_INTAKE_TEMP, PID_COOLANT_TEMP, PID_DISTANCE};
  static byte index = 0;
  byte pid = pids[index];
  int value;
  // send a query to OBD adapter for specified OBD-II pid
  if (obd.readPID(pid, value)) {
      showData(pid, value);
  }
  index = (index + 1) % sizeof(pids);

  if (obd.errors >= 2) {
      reconnect();
      setup();
  }
}
