#include <Arduino.h>
#include "config.h"
#include "thermocouples.h"
#include "encoder.h"
#include "display.h"
#include "pulse_control.h"

#define LED_BUILTIN 2

#define WindowSize 200
unsigned long nextTime, startTime, actualTime, windowStartTime; 

double tiempo;
double temperatura;
long int encoderCounter=0, oldEncoderCounter=0;

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(ZC_PIN, INPUT);
  
  digitalWrite(RELAY_PIN,HIGH);

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  encoder_init();
  initDisplay();
  // Inicializo los Timers que realizaran el pulso que activa el Triac 
  initPwmPulseSettings();
  setPwmPulse(encoderCounter);


  lcd.setTextSize(MIDLE_TEXT);
  lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
  lcd.print("Iniciando...");
  lcd.setTextSize(LITTLE_TEXT);

  encoder_setBasicParameters(0, 180, false, 90, 150);
  nextTime = actualTime + 200;

  encoderCounter = encoder_read();
  lcd.setCursor(1*6*BIG_TEXT, 1*8*BIG_TEXT+2);
  lcd.setTextSize(BIG_TEXT);
  lcd.printf("%4d", encoderCounter);
  
  while(!isButtonClicked()){
    if (actualTime > nextTime){ 
      readThermocouple(&temperatura);
      
      nextTime = actualTime + 200;
      
    }
    if(encoder_encoderChanged()){
      encoderCounter = encoder_read();
      if (oldEncoderCounter != encoderCounter) {
        lcd.setCursor(1*6*BIG_TEXT, 1*8*BIG_TEXT+2);
        lcd.setTextSize(BIG_TEXT);
        lcd.printf("%4d", encoderCounter);
        oldEncoderCounter = encoderCounter;
      }
    }
    actualTime = millis();
  }
  
  digitalWrite(LED_BUILTIN, HIGH);

  setPwmPulse(encoderCounter);

  lcd.fillScreen(ST7735_BLACK);
  lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
  lcd.setCursor(1*6*BIG_TEXT, 1*8*BIG_TEXT+2);
  lcd.setTextSize(BIG_TEXT);
  lcd.printf("%4d", encoderCounter);
  
  startTime = millis();
  windowStartTime = startTime;
  nextTime = startTime + WINDOW_1Seg;
}

void loop() {
  // Senso la temperatura cada 200mSeg 
  // (frecuencia máxima a la que lee el sensor max6675)
  if (millis() > windowStartTime){
    readThermocouple(&temperatura);

    windowStartTime = millis() + WindowSize;
  }

  if(encoder_encoderChanged()){
    encoderCounter = encoder_read();
    if (oldEncoderCounter != encoderCounter) {
      lcd.setCursor(1*6*BIG_TEXT, 1*8*BIG_TEXT+2);
      lcd.setTextSize(BIG_TEXT);
      lcd.printf("%4d", encoderCounter);
      oldEncoderCounter = encoderCounter;
      setPwmPulse(encoderCounter);
    }
  }

  // basic readout test, just print the current temp
  actualTime = millis();
  if (actualTime > nextTime){ 
    tiempo = (actualTime-startTime)/1000.0;
    
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setCursor(0*6*LITTLE_TEXT, 10*8*LITTLE_TEXT);
    lcd.print(temperatura);

    #ifdef DEBUG
    Serial.printf("$%d %d;",(uint16_t)tiempo, (uint16_t)temperatura);
    #endif
    
    nextTime = actualTime + WINDOW_1Seg;
    
  }
}