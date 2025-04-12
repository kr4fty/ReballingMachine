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
#define FONT_W                       6 //6 pixel de ancho
#define FONT_H                       8 //8 pixel de alto

#define XSCALE                       3 // Para reescalar en el eje X
#define YSCALE                       4 // Para reescalar en el eje Y
#define ORIGINX                     18 // Punto de origen de X, del grafico, en el LCD
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

void printPresentation()
{
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
    lcd.setTextSize(3);
    lcd.setCursor(0*FONT_W*3, 1*FONT_H*3);
    lcd.println("Reballin");
    lcd.print  (" Machine");

    lcd.setTextSize(MIDLE_TEXT);
    lcd.setCursor(0*FONT_W*MIDLE_TEXT, 5*FONT_H*MIDLE_TEXT);
    lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    lcd.print("Press Button");
}
void printLavelsInit()
{    
    lcd.setCursor(0*FONT_W*MIDLE_TEXT, 4*FONT_H*MIDLE_TEXT);
    lcd.print("   Setpoint:");
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(0*FONT_W*MIDLE_TEXT, 2*FONT_H*MIDLE_TEXT);
    lcd.print("Press Button");
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
}

void printLavels()
{
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(0*FONT_W*MIDLE_TEXT, 0*FONT_H*MIDLE_TEXT);
    lcd.print("Heater    Out");
    lcd.setCursor(0*FONT_W*MIDLE_TEXT, 2*FONT_H*MIDLE_TEXT);
    lcd.print("Board  Pulsos");
    lcd.setCursor(3*FONT_W*MIDLE_TEXT, 4*FONT_H*MIDLE_TEXT);
    lcd.print("Setpoint ");
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
}

void drawChart()
{
    lcd.fillRect(0, LCD_HEIGHT/2, LCD_WIDTH, LCD_HEIGHT/2, ST7735_BLACK);
    // Cuadriculado
    for(uint8_t x=ORIGINX; x<LCD_WIDTH; x+=(60/XSCALE)){
        for(uint8_t y=LCD_HEIGHT/2+2; y<=ORIGINY+2; y+=3){
            lcd.drawPixel(x, y, ST7735_CYAN); // Lineas punteadas verticales
        }
    }
    for(uint8_t x=ORIGINX-2; x<LCD_WIDTH; x+=3){
        for(uint8_t y=ORIGINY; y>=(LCD_HEIGHT/2); y-=(100/(2*YSCALE))){
            lcd.drawPixel(x, y, ST7735_CYAN); // Lineas punteadas horizontales
        }
    }
    // Imprimo escala
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setTextColor(ST7735_WHITE);
    lcd.setCursor(0, LCD_HEIGHT-8*LITTLE_TEXT);
    lcd.print("  0");
    lcd.setCursor(0, ORIGINY-2*(100/(2*YSCALE))-(LITTLE_TEXT*FONT_H/2));
    lcd.print("100");
    lcd.setCursor(0, ORIGINY-4*(100/(2*YSCALE))-(LITTLE_TEXT*FONT_H/2));
    lcd.print("200");
    lcd.setCursor(6/2*LITTLE_TEXT, (LCD_HEIGHT/2)-(LITTLE_TEXT*FONT_H));
    lcd.setTextColor(ST7735_YELLOW);
    lcd.print((char)248); // 'º'
    lcd.print("C");

    // Dibujo los ejes
    lcd.drawLine(ORIGINX, LCD_HEIGHT/2-5, ORIGINX, ORIGINY, ST7735_WHITE); // eje Vertical
    lcd.drawLine(ORIGINX, ORIGINY, LCD_WIDTH-5, ORIGINY, ST7735_WHITE); // eje Horizontal
    // DIbujo las flechas
    lcd.drawLine(ORIGINX, LCD_HEIGHT/2-5-5, ORIGINX-2,  LCD_HEIGHT/2-5, ST7735_WHITE);
    lcd.drawLine(ORIGINX, LCD_HEIGHT/2-5-5, ORIGINX+2,  LCD_HEIGHT/2-5, ST7735_WHITE);
    lcd.drawLine(LCD_WIDTH-1, ORIGINY, LCD_WIDTH-1-5, ORIGINY-2, ST7735_WHITE);
    lcd.drawLine(LCD_WIDTH-1, ORIGINY, LCD_WIDTH-1-5, ORIGINY+2, ST7735_WHITE);

    lcd.setTextSize(MIDLE_TEXT);
}

// Frame principal una vez arrancado el Perfil seleccionado
void printFrameBase(uint16_t *posX, uint16_t *posY, uint8_t length)
{
    uint16_t posx[20], posy[20];
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(0*FONT_W*LITTLE_TEXT, 0*FONT_H*LITTLE_TEXT);
    lcd.print("Heater               Board");
    lcd.setCursor(0*FONT_W*LITTLE_TEXT, 3*FONT_H*LITTLE_TEXT);
    lcd.print("Status              Pulsos");

    drawChart();

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
void printPoint(uint16_t posx, uint16_t posy, uint16_t color=ST7735_YELLOW, double xScale=XSCALE, double yScale=YSCALE)
{
    // Escalamos y trasladamos los puntos
    if(xScale>1){
        posx = (uint16_t)(posx/xScale);
    }
    if(yScale>1){
        posy = (uint16_t)(posy/yScale);
    }

    if(posx>=140){
        if((posx%140)==0){
            drawChart();
        }
        posx = (uint16_t)(posx%140);
    }

    posy = ORIGINY - posy;
    posx = ORIGINX + posx;

    // Dibujamos el punto
    lcd.drawPixel(posx, posy, color);


}

void printProfiles(char profiles[][50], uint8_t profileLength)
{
    //Serial.printf("Perfiles: %d", profileLength);
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setCursor(3*FONT_W*MIDLE_TEXT, 0*FONT_H*MIDLE_TEXT);
    lcd.printf("PERFILES");
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setTextSize(LITTLE_TEXT);
    for(uint8_t i=1; i<=profileLength; i++){
        lcd.setCursor(3*FONT_W*LITTLE_TEXT, (1+2*i)*FONT_H*LITTLE_TEXT);
        lcd.printf("%s", profiles[i-1]);
        //Serial.println("Imprimo perfil");
    }

    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
}

void clearProfileSelection(uint8_t posY)
{
    lcd.setCursor(1*FONT_W*LITTLE_TEXT, (1*FONT_H*MIDLE_TEXT)+((1+2*posY)*FONT_H*LITTLE_TEXT)); //(1+2*posY)*8*LITTLE_TEXT)
    lcd.print(" ");
}

void printProfileSelection(uint8_t posY)
{
    lcd.setCursor(1*FONT_W*LITTLE_TEXT, (1*FONT_H*MIDLE_TEXT)+((1+2*posY)*FONT_H*LITTLE_TEXT)); //(1+2*posY)*8*LITTLE_TEXT)
    lcd.print(">"); // '»'
}

void printSelectedProfile(const char *name, uint16_t *posX, uint16_t *posY, uint8_t length)
{
    uint16_t posx[20], posy[20];
    lcd.fillScreen(ST7735_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(2*FONT_W*MIDLE_TEXT, 0*FONT_H*MIDLE_TEXT);
    lcd.print(name);

    // Cuadriculado
    for(uint8_t x=ORIGINX; x<(LCD_WIDTH); x+=(60/XSCALE)){
        for(uint8_t y=MIDLE_TEXT*FONT_H; y<=ORIGINY+2; y+=3){
            lcd.drawPixel(x, y, ST7735_CYAN); // Lineas punteadas verticales
        }
    }
    for(uint8_t x=ORIGINX-2; x<LCD_WIDTH-10; x+=3){
        for(uint8_t y=ORIGINY; y>=MIDLE_TEXT*FONT_H; y-=(100/(2*XSCALE))){
            lcd.drawPixel(x, y, ST7735_CYAN); // Lineas punteadas horizontales
        }
    }
    // Dibujo los ejes
    lcd.drawLine(ORIGINX, MIDLE_TEXT*FONT_H, ORIGINX, ORIGINY, ST7735_BLUE); // eje Vertical
    lcd.drawLine(ORIGINX, ORIGINY, LCD_WIDTH-5, ORIGINY, ST7735_BLUE); // eje Horizontal
    // DIbujo las flechas
    lcd.drawLine(ORIGINX, MIDLE_TEXT*FONT_H, ORIGINX-2, MIDLE_TEXT*FONT_H+5, ST7735_BLUE);
    lcd.drawLine(ORIGINX, MIDLE_TEXT*FONT_H, ORIGINX+2, MIDLE_TEXT*FONT_H+5, ST7735_BLUE);
    lcd.drawLine(LCD_WIDTH-5, ORIGINY, LCD_WIDTH-5-5, ORIGINY-2, ST7735_BLUE);
    lcd.drawLine(LCD_WIDTH-5, ORIGINY, LCD_WIDTH-5-5, ORIGINY+2, ST7735_BLUE);

    // Escalamos el Perfil a utilizar
    for(uint8_t i=0; i<length; i++){
        posy[i] = ORIGINY - posY[i]/XSCALE;
        posx[i] = ORIGINX + posX[i]/XSCALE;
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
    lcd.setCursor(2*FONT_W*BIG_TEXT, 3*FONT_H*BIG_TEXT);
    lcd.printf("%3d", value);
    lcd.setTextSize(MIDLE_TEXT);
}
void printInputs(double input1, double input2)
{
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(0*FONT_W*MIDLE_TEXT, 1*FONT_H*MIDLE_TEXT);
    lcd.printf("%3d",(uint16_t)input1);
    lcd.setCursor(0*FONT_W*MIDLE_TEXT, 3*FONT_H*MIDLE_TEXT);
    lcd.printf("%3d",(uint16_t)input2);
}
void printTemperatures(double input1, double input2)
{
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(0*FONT_W*MIDLE_TEXT, 1*FONT_H*LITTLE_TEXT);
    lcd.printf("%3d",(uint16_t)input1);
    lcd.setCursor(10*FONT_W*MIDLE_TEXT, 1*FONT_H*LITTLE_TEXT);
    lcd.printf("%3d",(uint16_t)input2);
}
void printOutputs(double output1, double output2)
{
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(9*FONT_W*MIDLE_TEXT, 1*FONT_H*MIDLE_TEXT);
    lcd.printf("%4d",(uint16_t)output1);
    lcd.setCursor(10*FONT_W*MIDLE_TEXT, 3*FONT_H*MIDLE_TEXT);
    lcd.printf("%3d",(uint16_t)output2);
}
void printZcCount(uint8_t zcCount)
{
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setCursor(9*FONT_W*MIDLE_TEXT, 4*FONT_H*LITTLE_TEXT);
    lcd.printf("%4d",(uint16_t)zcCount);
}

void printActualSetpoint(uint16_t value)
{
    lcd.setCursor(2*FONT_W*BIG_TEXT, 3*7*BIG_TEXT);
    lcd.setTextSize(BIG_TEXT);
    lcd.printf("%3d", value);
    lcd.setTextSize(MIDLE_TEXT);
}

void printSystemStatus(bool state)
{
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.setCursor(0*FONT_W*MIDLE_TEXT, 4*FONT_H*LITTLE_TEXT);
    
    if(state){ // RUN
        lcd.print(F(" "));
        lcd.print((char)175);   // '>>'
    }
    else{ //Pause
        lcd.print((char)222);   // '|'
        lcd.print((char)222);   // '|'
    }
}

void printTicks(uint16_t value)
{
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setCursor(0*FONT_W*LITTLE_TEXT, 15*FONT_H*LITTLE_TEXT);
    lcd.printf("%4d %c",value, digitalRead(RELAY_PIN)?'H':'L');
    lcd.setTextSize(MIDLE_TEXT);
}

void printPulsesStatus(bool status)
{
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setCursor(0*FONT_W*LITTLE_TEXT, 15*FONT_H*LITTLE_TEXT);
    lcd.printf("%s", status?" OK ":"Fail");
    lcd.setTextSize(MIDLE_TEXT);
}

void printProfileName(const char *name, uint16_t color=ST7735_WHITE)
{
    uint8_t xCenter = LCD_WIDTH/(2*FONT_W)-strlen(name)/2;
    //Serial.print(xCenter);
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setCursor(xCenter*FONT_W*LITTLE_TEXT, 6*FONT_H*LITTLE_TEXT);
    lcd.setTextColor(color, ST7735_BLACK);
    lcd.print(name);

    lcd.setTextSize(MIDLE_TEXT);
}

void printTime(uint8_t hs, uint8_t min, uint8_t seg, uint16_t color=ST7735_WHITE)
{
    uint8_t xCenter = LCD_WIDTH/(2*FONT_W)-8/2; //HH:MM:SS

    lcd.setTextSize(LITTLE_TEXT);
    lcd.setCursor(xCenter*FONT_W*LITTLE_TEXT, 3*FONT_H*LITTLE_TEXT);
        
    lcd.setTextColor(color, ST7735_BLACK);
    lcd.printf("%02d:%02d:%02d",hs,min,seg);

    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
}

#endif