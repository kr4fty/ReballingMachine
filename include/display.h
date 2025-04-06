/******************************************************************************
 * 
 *  display.h
 * 
 *  Se encarga del manejo del display, ya sea tanto la inicializacion como asi
 *  también de mostrar parámetros (Temperaturas, Angulo de disparo, detección
 *  de cruces por cero, etc.)
 * 
 *  ST7735
 *          128x160
 * 
 * 
 * 
 *  Autor: Tapia Velasquez Favio
 * 
 *****************************************************************************/

#ifndef DISPLAY_H
#define DISPLAY_H
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "config.h"

Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void initDisplay()
{
    lcd.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab // Para sobre escribir puntos
    lcd.cp437(true);
    lcd.setRotation(3);
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

void printPresentation()
{
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
    lcd.setTextSize(3);
    lcd.setCursor(0*6*3, 1*8*3);
    lcd.println("Reballin");
    lcd.print  (" Machine");

    lcd.setTextSize(MIDLE_TEXT);
    lcd.setCursor(0*6*MIDLE_TEXT, 5*8*MIDLE_TEXT);
    lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    lcd.print("Press Button");
}
void printLavelsInit()
{    
    lcd.setCursor(0*6*MIDLE_TEXT, 4*8*MIDLE_TEXT);
    lcd.print("   Setpoint:");
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
    lcd.print("Press Button");
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
}

void printLavels()
{
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.print("Heater    Out");
    lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
    lcd.print("Board  Pulsos");
    lcd.setCursor(3*6*MIDLE_TEXT, 4*8*MIDLE_TEXT);
    lcd.print("Setpoint ");
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
}

void printProfiles(char profiles[][50], uint8_t profileLength)
{
    //Serial.printf("Perfiles: %d", profileLength);
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(3*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.printf("PERFILES");
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setTextSize(LITTLE_TEXT);
    for(uint8_t i=1; i<=profileLength; i++){
        lcd.setCursor(3*6*LITTLE_TEXT, (1+2*i)*8*LITTLE_TEXT);
        lcd.printf("%s", profiles[i-1]);
        //Serial.println("Imprimo perfil");
    }

    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
}

void clearProfileSelection(uint8_t posY)
{
    lcd.setCursor(1*6*LITTLE_TEXT, (1+2*posY)*8*LITTLE_TEXT);
    lcd.print(" ");
}

void printProfileSelection(uint8_t posY)
{
    lcd.setCursor(1*6*LITTLE_TEXT, (1+2*posY)*8*LITTLE_TEXT);
    lcd.print(">");
}

void printSelectedProfile(char *name)
{
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(1*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.print("SELECCIONADO");
    lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.print(name);
    delay(5000);
}

void printEncoderValue(long value)
{
    lcd.setTextSize(BIG_TEXT);
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(2*6*BIG_TEXT, 3*8*BIG_TEXT);
    lcd.printf("%3d", value);
    lcd.setTextSize(MIDLE_TEXT);
}
void printInputs(double input1, double input2)
{
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(0*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
    lcd.printf("%3d",(uint16_t)input1);
    lcd.setCursor(0*6*MIDLE_TEXT, 3*8*MIDLE_TEXT);
    lcd.printf("%3d",(uint16_t)input2);
}
void printOutputs(double output1, double output2)
{
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(9*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
    lcd.printf("%4d",(uint16_t)output1);
    lcd.setCursor(10*6*MIDLE_TEXT, 3*8*MIDLE_TEXT);
    lcd.printf("%3d",(uint16_t)output2);
}

void printActualSetpoint(uint16_t value)
{
    lcd.setCursor(2*6*BIG_TEXT, 3*7*BIG_TEXT);
    lcd.setTextSize(BIG_TEXT);
    lcd.printf("%3d", value);
    lcd.setTextSize(MIDLE_TEXT);
}

void printSystemStatus(bool state)
{
    lcd.setTextSize(MIDLE_TEXT);
    lcd.setCursor(0*MIDLE_TEXT*6, 6*MIDLE_TEXT*8);
    
    if(state){ // RUN
        lcd.print(F(" "));
        lcd.print((char)175);   // '>>'
    }
    else{ //Pause
        lcd.print((char)222);   // 'l'
        lcd.print((char)222);   // '|'
    }
    lcd.setTextSize(MIDLE_TEXT);
}

void printTicks(uint16_t value)
{
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setCursor(0*LITTLE_TEXT*6, 15*LITTLE_TEXT*8);
    lcd.printf("%4d %c",value, digitalRead(RELAY_PIN)?'H':'L');
    lcd.setTextSize(MIDLE_TEXT);
}

void printPulsesStatus(bool status)
{
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setCursor(0*LITTLE_TEXT*6, 15*LITTLE_TEXT*8);
    lcd.printf("%s", status?" OK ":"Fail");
    lcd.setTextSize(MIDLE_TEXT);
}

#endif