#ifndef DISPLAY_H
#define DISPLAY_H
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "config.h"

Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void initDisplay(){
    lcd.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab // Para sobre escribir puntos
    lcd.cp437(true);
    lcd.setRotation(1);
    lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    lcd.fillScreen(ST77XX_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
}

/*void printDisplay(class Adafruit_ST7735 &lcd,
                  uint8_t text_size,
                  uint8_t x_pos,
                  uint8_t y_pos,
                  int color_front,
                  int color_back,
                  char *text){
    
    lcd.setTextSize(text_size);
    lcd.setCursor(x_pos*6*text_size, y_pos*8*text_size);
    lcd.setTextColor(color_front, color_back);
    lcd.print(text);
}*/
void printLavelsInit(){    
    lcd.setCursor(0*6*MIDLE_TEXT, 4*8*MIDLE_TEXT);
    lcd.print("   Setpoint:");
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
    lcd.print("Press Button");
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
}

void printLavels(){
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.print("Heater    Out");
    lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
    lcd.print("Board  Pulsos");
    lcd.setCursor(3*6*MIDLE_TEXT, 4*8*MIDLE_TEXT);
    lcd.print("Setpoint ");
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
}
void printEncoderValue(long value){
    lcd.setTextSize(BIG_TEXT);
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(2*6*BIG_TEXT, 3*8*BIG_TEXT);
    lcd.printf("%3d", value);
    lcd.setTextSize(MIDLE_TEXT);
}
void printInputs(double input1, double input2){
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(0*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
    lcd.printf("%3d",(uint16_t)input1);
    lcd.setCursor(0*6*MIDLE_TEXT, 3*8*MIDLE_TEXT);
    lcd.printf("%3d",(uint16_t)input2);
}
void printOutputs(double output1, double output2){
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(9*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
    lcd.printf("%4d",(uint16_t)output1);
    lcd.setCursor(10*6*MIDLE_TEXT, 3*8*MIDLE_TEXT);
    lcd.printf("%3d",(uint16_t)output2);
}
#endif