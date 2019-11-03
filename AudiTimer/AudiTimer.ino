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
#include<U8g2lib.h>



//Set parameters for the screen
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_SLEEPING 0x20

static int lastSpeed = -1;
static uint32_t lastSpeedTime = 0;
static int speed = 0;
static uint32_t distance = 0;
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
      u8g2.clearBuffer();
			do {
				showStates();
			} while (!init());
			state |= STATE_OBD_READY;
			showStates();
			initTimerScreen();
      u8g2.sendBuffer();
    }
    void loop()
    {
			static byte index = 0;
			static byte index2 = 0;
			static byte index3 = 0;
			timerLoop();
		//recpnnect if errors
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
        u8g2.setFont(u8g2_font_profont10_mf);
        u8g2.setCursor(97, 7);
        u8g2.print(buf);
    }
    void timerLoop()
    {
        uint32_t elapsed = millis() - startTime;
        uint16_t n;
        int speed;
        uint32_t dataTime;
        if (!readPID(PID_SPEED, speed))
            return;
        dataTime = millis();
        u8g2.setFont(u8g2_font_profont17_mf);
        // estimate distance
        distance += (uint32_t)(speed + lastSpeed) * (dataTime - lastSpeedTime) / 2 / 3600;

        if (lastSpeed != speed) {
            u8g2.setCursor(0, 4);
            u8g2.print((unsigned int)speed % 1000, 3);
            lastSpeed = speed;
        }

        lastSpeedTime = dataTime;

        if (stage == STAGE_WAIT_START) {
            if (speed > 0) {
                stage = STAGE_MEASURING;
                startTime = lastSpeedTime;
                lastSpeed = 0;
                distance = 0;
                //Clear Array
                memset(times, 0, sizeof(times));
                initTimerScreen();
            }
        } else if (stage == STAGE_MEASURING) {
            // display elapsed time (mm:ss:mm)
            n = elapsed / 1000;
            if (n < 100) {
                u8g2.setCursor(0, 0);
                u8g2.print(n);
                n = (elapsed % 1000) / 100;
                u8g2.setFont(u8g2_font_profont12_mf);
                //u8g2.setCursor(32, 1);
                u8g2.drawStr(32,1,".");
                u8g2.drawStr(32,1,"0" + n);
            }
            if (times[2] == 0 && speed >= SPEED_THRESHOLD_3) {
                times[2] = elapsed / 100;
                stage = STAGE_IDLE;
                //u8g2.clear(0, 0, 128, 24);
                showTimerResults();
                u8g2.setFont(u8g2_font_profont12_mf);
                u8g2.setCursor(0, 0);
                u8g2.print("DONE!");
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
                u8g2.setFont(u8g2_font_profont10_mf);
                //lcd.setCursor(62, 6);
                if (distance >= 400) {
                    u8g2.drawStr(62, 6, "400");
                    if (!times[3]) {
                        times[3] = elapsed / 100;
                        showTimerResults();
                    }
                } else {
                    u8g2.print(distance);
                }
            }
        } else {
            if (speed == 0) {
                stage = STAGE_WAIT_START;
                initTimerScreen();
                u8g2.setFont(u8g2_font_profont10_mf);
                u8g2.setCursor(0, 0);
                u8g2.println(" GET");
                u8g2.println("READY");
                delay(500);
            }
        }
    }
    void reconnect()
    {
        u8g2.clear();
        u8g2.setFont(u8g2_font_roentgen_nbp_t_all);
        u8g2.drawStr(0,15,"Reconnecting");
        startTime = millis();
        state &= ~STATE_OBD_READY;
        state |= STATE_SLEEPING;
        //digitalWrite(SD_CS_PIN, LOW);
        for (int i = 0; !init(); i++) {
            if (i == 10) u8g2.clear();
        }
        state &= ~STATE_SLEEPING;
        setup();
    }
    byte state;

    // screen layout related stuff
    void showStates()
    {
        if (state & STATE_OBD_READY) {
            u8g2.setFont(u8g2_font_profont10_mf);
            u8g2.drawStr(0,50, "OBD connected!   ");
        } else {
            u8g2.setFont(u8g2_font_profont10_mf);
            u8g2.drawStr(0,50, "Connecting OBD...");
        }
    }
    void showTimerResults()
    {
        u8g2.setFont(u8g2_font_profont10_mf);
        u8g2.setCursor(56, 0);
        u8g2.print(" 0~60:  --");
        u8g2.setCursor(56, 2);
        u8g2.print("0~100:  --");
        u8g2.setCursor(56, 4);
        u8g2.print("0~200:  --");
        u8g2.setCursor(56, 6);
        u8g2.print(" 400m:  --");
        u8g2.setFont(u8g2_font_profont12_mf);
        char buf[8];
        if (times[0]) {
            sprintf(buf, "%2d.%1d", times[0] / 10, times[0] % 10);
            Serial.println(times[0]);
            u8g2.setCursor(92, 0);
            u8g2.print(buf);
        }
        if (times[1]) {
            sprintf(buf, "%2d.%1d", times[1] / 10, times[1] % 10);
            Serial.println(buf);
            u8g2.setCursor(92, 2);
            u8g2.print(buf);
        }
        if (times[2]) {
            sprintf(buf, "%2d.%1d", times[2] / 10, times[2] % 10);
            Serial.println(buf);
            u8g2.setCursor(92, 4);
            u8g2.print(buf);
        }
        if (times[3]) {
            sprintf(buf, "%2d.%1d", times[3] / 10, times[3] % 10);
            Serial.println(buf);
            u8g2.setCursor(92, 6);
            u8g2.print(buf);
        }
    }
    void initTimerScreen()
    {
        u8g2.clear();
        showTimerResults();
        u8g2.setFont(u8g2_font_profont10_mf);
        u8g2.setCursor(24, 7);
        u8g2.print("km/h");
    }
};

static COBDLogger logger;

void setup()
{
    
    u8g2.begin();
    delay(100);
    u8g2.clearBuffer();
    //u8g2.setFont(u8g2_font_profont10_mf);
    //u8g2.setCursor(0, 30);
    //u8g2.println("PerformanceBox");
     logger.setup();
     u8g2.sendBuffer();
    
}

void loop()
{
    u8g2.clearBuffer();
    logger.loop();
    u8g2.sendBuffer();
}
