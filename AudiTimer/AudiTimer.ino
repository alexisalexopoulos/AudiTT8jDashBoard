/*************************************************************************
* OBD-II based performance timer and logger
* Distributed under GPL v2.0
* Copyright (c) 2014 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <MicroLCD.h>
#include <OBD2UART.h>
#include "config.h"
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_SLEEPING 0x20

static uint32_t lastFileSize = 0;
static int lastSpeed = -1;
static uint32_t lastSpeedTime = 0;
static int speed = 0;
static uint32_t distance = 0;
static uint16_t fileIndex = 0;
static uint32_t startTime = 0;

#define STAGE_IDLE 0
#define STAGE_WAIT_START 1
#define STAGE_MEASURING 2

static byte stage = STAGE_IDLE;

#define SPEED_THRESHOLD_1 60 /* kph */
#define SPEED_THRESHOLD_2 100 /* kph */
#define SPEED_THRESHOLD_3 200 /* kph */
#define DISTANCE_THRESHOLD 400 /* meters */

static uint16_t times[4] = {0};


class COBDLogger : public COBD
{
public:
    COBDLogger():state(0) {}
    void setup()
    {
        do {
            showStates();
        } while (!init());

        state |= STATE_OBD_READY;

       showStates();


        initTimerScreen();
    }
    void loop()
    {
        static byte index = 0;
        static byte index2 = 0;
        static byte index3 = 0;

        timerLoop();


        if (errors >= 2) {
            reconnect();
        }
    }

private:
    void dataIdleLoop()
    {
        if (state & STATE_SLEEPING) return;

        if (getState() == OBD_CONNECTED)
            return;

        // called while initializing
        char buf[10];
        unsigned int t = (millis() - startTime) / 1000;
        sprintf(buf, "%02u:%02u", t / 60, t % 60);
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setCursor(97, 7);
        lcd.print(buf);
    }
    void timerLoop()
    {
        uint32_t elapsed = millis() - startTime;
        uint16_t n;

        int speed;
        if (!readPID(PID_SPEED, speed))
            return;

        dataTime = millis();


        lcd.setFontSize(FONT_SIZE_XLARGE);
        // estimate distance
        distance += (uint32_t)(speed + lastSpeed) * (dataTime - lastSpeedTime) / 2 / 3600;

        if (lastSpeed != speed) {
            lcd.setCursor(0, 4);
            lcd.printInt((unsigned int)speed % 1000, 3);
            lastSpeed = speed;
        }

        lastSpeedTime = dataTime;

        if (stage == STAGE_WAIT_START) {
            if (speed > 0) {
                stage = STAGE_MEASURING;
                startTime = lastSpeedTime;



                lastSpeed = 0;
                distance = 0;

                memset(times, 0, sizeof(times));

                initTimerScreen();
            }
        } else if (stage == STAGE_MEASURING) {
            // display elapsed time (mm:ss:mm)
            n = elapsed / 1000;
            if (n < 100) {
                lcd.setCursor(0, 0);
                lcd.printInt(n, 2);
                n = (elapsed % 1000) / 100;
                lcd.setFontSize(FONT_SIZE_MEDIUM);
                lcd.setCursor(32, 1);
                lcd.write('.');
                lcd.write('0' + n);
            }
            if (times[2] == 0 && speed >= SPEED_THRESHOLD_3) {
                times[2] = elapsed / 100;
                stage = STAGE_IDLE;
                lcd.clear(0, 0, 128, 24);
                showTimerResults();
                lcd.setFontSize(FONT_SIZE_MEDIUM);
                lcd.setCursor(0, 0);
                lcd.print("DONE!");
            } else if (times[1] == 0 && speed >= SPEED_THRESHOLD_2) {
                times[1] = elapsed / 100;
                showTimerResults();
            } else if (times[0] == 0 && speed >= SPEED_THRESHOLD_1) {
                times[0] = elapsed / 100;
                showTimerResults();
            } else if (speed == 0) {
                // speed go back to 0
                stage = STAGE_IDLE;
            }
            if (distance > 0) {
                lcd.setFontSize(FONT_SIZE_SMALL);
                lcd.setCursor(62, 6);
                if (distance >= 400) {
                    lcd.printInt(400, 3);
                    if (!times[3]) {
                        times[3] = elapsed / 100;
                        showTimerResults();
                    }
                } else {
                    lcd.printInt(distance, 3);
                }
            }


            if (speed == 0) {
                stage = STAGE_WAIT_START;
                initTimerScreen();
                lcd.setFontSize(FONT_SIZE_MEDIUM);
                lcd.setCursor(0, 0);
                lcd.println(" GET");
                lcd.println("READY");
                delay(500);
            }
        }
    }
    void reconnect()
    {

        lcd.clear();
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.print("Reconnecting");
        startTime = millis();
        state &= ~STATE_OBD_READY;
        state |= STATE_SLEEPING;
        //digitalWrite(SD_CS_PIN, LOW);
        for (int i = 0; !init(); i++) {
            if (i == 10) lcd.clear();
        }
        state &= ~STATE_SLEEPING;
        fileIndex++;
        setup();
    }
    byte state;

    // screen layout related stuff
    void showStates()
    {
        u8g2.setFont(u8g2_font_profont15_mf);
        u8g2.setCursor(0, 3);
        if (state & STATE_OBD_READY) {
            u8g2.println("OBD connected!   ");
        } else {
            u8g2.println("Connecting OBD...");
        }
    }
    void showTimerResults()
    {
        do{
            u8g2.setFont(u8g2_font_profont10_mf);
            u8g2.firstPage();
            u8g2.drawStr(56,0," 0~60:  --")
            u8g2.drawStr(56,2,"0~100:  --")
            u8g2.drawStr(56,4,"0~200:  --")
            u8g2.drawStr(56,6," 400m:  --")
            char buf[8];
            if (times[0]) {
                sprintf(buf, "%2d.%1d", times[0] / 10, times[0] % 10);
                Serial.println(times[0]);
                u8g2.setCursor(92, 0);
                u8g2.print(buf);
            }
            if (times[1]) {
                sprintf(buf, "%2d.%1d", times[0] / 10, times[0] % 10);
                Serial.println(times[0]);
                u8g2.setCursor(92, 2);
                u8g2.print(buf);
            }
            if (times[2]) {
                sprintf(buf, "%2d.%1d", times[2] / 10, times[2] % 10);
                Serial.println(buf);
                lcd.setCursor(92, 4);
                lcd.print(buf);
                sprintf(buf, "%2d.%1d", times[0] / 10, times[0] % 10);
                Serial.println(times[0]);
                u8g2.setCursor(92, 4);
                u8g2.print(buf);
            }
            if (times[3]) {
                sprintf(buf, "%2d.%1d", times[0] / 10, times[0] % 10);
                Serial.println(times[0]);
                u8g2.setCursor(92, 6);
                u8g2.print(buf);
            }
        }
        } while ( u8g2.nextPage() );

        void initTimerScreen()
        {
            u8g2.clear();
            showTimerResults();
            u8g2.setFont(u8g2_font_profont10_mf);
            u8g2.drawStr(24,7,"km/h")
        }
};

static COBDLogger logger;

void setup()
{

    u8g2.begin();
    logger.begin();
    logger.setup();
}

void loop()
{
    logger.loop();
}
