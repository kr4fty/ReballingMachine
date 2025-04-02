// this example is public domain. enjoy!
// https://learn.adafruit.com/thermocouple/

#include <Arduino.h>
#include "config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <max6675.h>

#define WindowSize 200
uint32_t windowStartTime;
uint32_t  nextTime;

float Input1,Input2,InputFiltered1,InputFiltered2;

MAX6675 thermocouple1(TH1_CS);
MAX6675 thermocouple2(TH2_CS);
Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  // DEBUG
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  
  lcd.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Para sobre escribir puntos
  lcd.fillScreen(ST77XX_BLACK);
  lcd.setRotation(1);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
  lcd.print("Temp 1:");
  lcd.setCursor(0*6*MIDLE_TEXT, 4*8*MIDLE_TEXT);
  lcd.print("Temp 2:");
  lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);

  windowStartTime = millis();
  nextTime=millis() + WINDOW_1Seg;
}
float Timed;
void loop() {
  // basic readout test, just print the current temp

  if (millis() > windowStartTime){
      Input1 = thermocouple1.readCelsius();
      Input2 = thermocouple2.readCelsius();
      #ifdef ALPHA
      InputFiltered1 = (ALPHA*Input1) + ((1-ALPHA)*InputFiltered1);
      Input1 = InputFiltered1;
      InputFiltered2 = (ALPHA*Input2) + ((1-ALPHA)*InputFiltered2);
      Input2 = InputFiltered2;
      #endif
      Timed=millis()-windowStartTime+WindowSize;
      Serial.printf("%.2f %.2f %.2f\n", Input1, Input2,Timed);
      windowStartTime = millis() + WindowSize;
  }

  if(millis()>nextTime){
    lcd.setCursor(7*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
    lcd.printf("%.2f", Input1);
    lcd.setCursor(7*6*MIDLE_TEXT, 4*8*MIDLE_TEXT);
    lcd.printf("%.2f", Input2);
    // DEBUG
    #ifdef DEBUG
    //Serial.printf("%.2f %.2f\n",Input1, Input2);
    #endif

    nextTime=millis() + WINDOW_1Seg;      
  }
}
