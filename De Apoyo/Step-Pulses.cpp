#include <Arduino.h>
#include "config.h"
#include "thermocouples.h"
#include "encoder.h"
#include "display.h"

#define LED_BUILTIN 2

#define WindowSize 200
unsigned long nextTime, startTime, actualTime, windowStartTime; 

double tiempo;
double temperatura;
long int encoderCounter=230, oldEncoderCounter=0;

void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(COOLER_PIN, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(ZC_PIN, INPUT);
  
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(COOLER_PIN, LOW);

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  encoder_init();

  initDisplay();

  nextTime = actualTime + 200;
  while(!isButtonClicked()){
    if (actualTime > nextTime){ 
      readThermocouple(&temperatura);

      lcd.setTextSize(LITTLE_TEXT);
      lcd.setCursor(0*6*LITTLE_TEXT, 10*8*LITTLE_TEXT);
      lcd.print(temperatura);
      
      nextTime = actualTime + 200;
    }
    actualTime = millis();
  }
  
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(HEATER_PIN, HIGH);

  lcd.fillScreen(ST7735_BLACK);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
  lcd.printf("Enviando\n    Datos >>>");
  lcd.setTextSize(LITTLE_TEXT);
  
  startTime = millis();
  windowStartTime = nextTime;
  nextTime = startTime + WindowSize;
}

void loop() {
  actualTime = millis();
  // Senso la temperatura cada 200mSeg 
  // (frecuencia máxima a la que lee el sensor max6675)
  if (actualTime > windowStartTime){
    readThermocouple(&temperatura);

    windowStartTime = actualTime + WindowSize;
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
  
  #ifdef DEBUG
  // basic readout test, just print the current temp
  actualTime = millis();
  if (actualTime > nextTime){ 
    tiempo = (actualTime-startTime)/1000.0;
    
    lcd.setCursor(0*6*LITTLE_TEXT, 10*8*LITTLE_TEXT);
    lcd.print(temperatura);
    Serial.printf("$%.2f %d;",tiempo, (uint16_t)temperatura);
    
    nextTime = actualTime + WindowSize;
    
  }
  #endif
}