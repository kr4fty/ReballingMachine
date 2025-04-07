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

// Al usarlo rotado los valores estan dado vuelta
#define LCD_HEIGHT ST7735_TFTWIDTH_128 // 128
#define LCD_WIDTH ST7735_TFTHEIGHT_160 // 160
#define XSCALE                       3 // Para reescalar en el eje X
#define YSCALE                       4 // Para reescalar en el eje Y
#define ORIGINX                     10 // Punto de origen de X, del grafico, en el LCD
#define ORIGINY           LCD_HEIGHT-4 // Punto de origen de Y, del grafico, en el LCD

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

// Frame principal una vez arrancado el Perfil seleccionado
void printFrameBase(uint16_t *posX, uint16_t *posY, uint8_t length)
{
    uint16_t posx[20], posy[20];
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.print("Heater  Board");
    lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
    lcd.print("Status Pulsos");

    // Cuadriculado
    /*for(uint8_t i=ORIGINX; i<(LCD_WIDTH); i+=(60/XSCALE)){
        lcd.drawLine(i, LCD_HEIGHT/2+2, i, ORIGINY+2, ST7735_CYAN); // Lineas verticales
    }
    for(uint8_t i=ORIGINY; i>=(LCD_HEIGHT/2); i-=(100/(2*YSCALE))){
        lcd.drawLine(ORIGINX-2, i, LCD_WIDTH-10, i, ST7735_CYAN); // Lineas horizontales
    }*/
    for(uint8_t x=ORIGINX; x<(LCD_WIDTH); x+=(60/XSCALE)){
        for(uint8_t y=LCD_HEIGHT/2+2; y<=ORIGINY+2; y+=3){
            lcd.drawPixel(x, y, ST7735_CYAN); // Lineas punteadas verticales
        }
    }
    for(uint8_t x=ORIGINX-2; x<LCD_WIDTH-10; x+=3){
        for(uint8_t y=ORIGINY; y>=(LCD_HEIGHT/2); y-=(100/(2*YSCALE))){
            lcd.drawPixel(x, y, ST7735_CYAN); // Lineas punteadas horizontales
        }
    }
    // Dibujo los ejes
    lcd.drawLine(ORIGINX, LCD_HEIGHT/2-5, ORIGINX, ORIGINY, ST7735_BLUE); // eje Vertical
    lcd.drawLine(ORIGINX, ORIGINY, LCD_WIDTH-5, ORIGINY, ST7735_BLUE); // eje Horizontal

    // Escalamos el Perfil a utilizar
    for(uint8_t i=0; i<length; i++){
        posy[i] = ORIGINY - posY[i]/YSCALE;
        posx[i] = ORIGINX + posX[i]/XSCALE;
    }
    // Dibujo el Perfil
    for(uint8_t i=1; i<length; i++){
        lcd.drawLine(posx[i-1],posy[i-1], posx[i], posy[i], ST7735_WHITE);
    }
}
void printPoint(uint16_t posx, uint16_t posy)
{
    // escalamos
    posy = ORIGINY - posy/YSCALE;
    posx = ORIGINX + posx/XSCALE;

    // Dibujamos el punto
    lcd.drawPixel(posx, posy, ST7735_YELLOW);
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
    lcd.print((char)175); // '>>'
}

void printSelectedProfile(const char *name, uint16_t *posX, uint16_t *posY, uint8_t length)
{
    uint16_t posx[20], posy[20];
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(2*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.print(name);

    // Dibujo los ejes
    lcd.drawFastVLine(5,  10, 109, ST7735_YELLOW);
    lcd.drawFastHLine(5, 120, 150, ST7735_YELLOW);
    // Escalar el Perfil
    for(uint8_t i=0; i<length; i++){
        posy[i] = 109 - posY[i]/YSCALE;
        posx[i] = 5 + posX[i]/XSCALE;
    }
    // Dibujo el Perfil
    for(uint8_t i=1; i<length; i++){
        lcd.drawLine(posx[i-1],posy[i-1], posx[i], posy[i], ST7735_WHITE);
    }
    
    delay(10000);
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
void printTemperatures(double input1, double input2)
{
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(0*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
    lcd.printf("%3d",(uint16_t)input1);
    lcd.setCursor(8*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
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
void printZcCount(uint8_t zcCount)
{
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(9*6*MIDLE_TEXT, 3*8*MIDLE_TEXT);
    lcd.printf("%4d",(uint16_t)zcCount);
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
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.setCursor(3*MIDLE_TEXT*6, 3*MIDLE_TEXT*8);
    
    if(state){ // RUN
        lcd.print(F(" "));
        lcd.print((char)175);   // '>>'
    }
    else{ //Pause
        lcd.print((char)222);   // 'l'
        lcd.print((char)222);   // '|'
    }
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