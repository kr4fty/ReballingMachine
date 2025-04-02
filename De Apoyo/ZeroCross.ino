/* Deteccion de cruce por cero implementado con interrupsiones
 *
 * Se cuenta cuantas interrupsiones hubo en 1 segundo
 *
 * AUTOR: Tapia Velasquez Favio
 */

#include <Arduino.h>
#include "config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <AiEsp32RotaryEncoder.h>

AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(ENCODER_PIN_A, ENCODER_PIN_B, BUTTON, ENCODER_VCC_PIN, ENCODER_STEPS);
void IRAM_ATTR readEncoderISR(){
	encoder.readEncoder_ISR();
}

#define WindowSize 200
unsigned long windowStartTime, startTime;
double Time;

uint32_t zcNextTime=0, nextTime; 
uint64_t pulseOn, pulseOff, pulseDelay=0;
bool zcFlag=false, pulseFlag=false, pulseStatus;
long int zcCounter=0;

Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void IRAM_ATTR zc_isr() {
  // Filtro para falsos positivos
  #ifdef ZC_INTERRUPT_FILTER
  if(!zcFlag){
    zcFlag = true;
    zcNextTime=millis()+WINDOW_INTERRUPT;
    pulseFlag = true;
    pulseOn = micros()+pulseDelay+ZC_PULSE_WIDTH/2;
  }
  #else
    zcCounter++;
    pulseFlag = true;
    pulseOn = micros()+pulseDelay+ZC_PULSE_WIDTH/2;
  #endif
}

void setup() {
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(ZC_PIN, INPUT);

  // Use this initializer if using a 1.8" TFT screen:
  lcd.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Para sobre escribir puntos
  lcd.fillScreen(ST77XX_BLACK);
  lcd.setRotation(1);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.setCursor(0*6*MIDLE_TEXT, 3*8*MIDLE_TEXT);
  lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  

  attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_isr,RISING);

  startTime = windowStartTime;
  nextTime=millis() + WINDOW_1Seg;

}

void loop(){
  // Filtro para falsos positivos
  #ifdef ZC_INTERRUPT_FILTER
  if(millis()>zcNextTime && zcFlag){
    zcFlag = false;
    zcCounter++;
  }
  #endif

  // cuento interrupsiones de cruce por cero, por segundo
  if(millis()>nextTime){
    #ifdef DEBUG
    //Time = (millis()-startTime)/1000.0;
    //Serial.printf("$%d;", encoderCounter);
    #endif

    lcd.setCursor(6*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.printf("%3d", zcCounter);
    zcCounter=0;
    nextTime=millis() + WINDOW_1Seg;

  }
}
 
