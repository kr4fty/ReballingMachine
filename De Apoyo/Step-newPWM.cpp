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
long int encoderCounter, oldEncoderCounter=-1;

void printAnguloDeDisparo(long value)
{
  lcd.setCursor(1*6*BIG_TEXT, 1*8*BIG_TEXT+2);
  lcd.setTextSize(BIG_TEXT);
  lcd.printf("%4d", value);
  lcd.setTextSize(LITTLE_TEXT);
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(ZC_PIN, INPUT);
  
  digitalWrite(RELAY_PIN,LOW);

  #if defined(DEBUG) || defined(SERIAL_PLOTTER)
  Serial.begin(115200);
  #endif

  encoder_init();

  initDisplay();

  initPwmPulseSettings();

  encoder_setBasicParameters(0, MAXIMUN_ANGLE, false, -1, 10);

  lcd.setCursor(1*6*BIG_TEXT, 1*8*BIG_TEXT+2);
  lcd.setTextSize(BIG_TEXT);
  lcd.printf("%4d", encoderCounter);

  nextTime = actualTime + 200;
  while(!isButtonClicked()){
    if (actualTime > nextTime){ 
      readThermocouple(&temperatura);

      lcd.setTextSize(LITTLE_TEXT);
      lcd.setCursor(0*6*LITTLE_TEXT, 10*8*LITTLE_TEXT);
      lcd.print(temperatura);
      
      nextTime = actualTime + 200;
    }
    if(encoder_encoderChanged()){
      encoderCounter = encoder_read();
      if (oldEncoderCounter != encoderCounter) {
        printAnguloDeDisparo(encoderCounter);
        oldEncoderCounter = encoderCounter;
      }
    }

    actualTime = millis();
  }
  
  digitalWrite(LED_BUILTIN, HIGH);

  lcd.fillScreen(ST7735_BLACK);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
  lcd.print("Capturando...");

  printAnguloDeDisparo(encoderCounter);

  setPwmPulse(LOWER_HEATER, encoderCounter);
  
  startTime = millis();
  windowStartTime = nextTime;
  nextTime = startTime + WINDOW_1Seg;
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
      printAnguloDeDisparo(encoderCounter);

      oldEncoderCounter = encoderCounter;
    }
  }
  
  #ifdef SERIAL_PLOTTER
  actualTime = millis();
  if (actualTime > nextTime){ 
    tiempo = (actualTime-startTime)/1000.0;
    
    lcd.setCursor(0*6*LITTLE_TEXT, 10*8*LITTLE_TEXT);
    lcd.print(temperatura);
    Serial.printf("$%d %d;",(uint16_t)tiempo, (uint16_t)temperatura);
    
    nextTime = actualTime + WINDOW_1Seg;
    
  }
  #endif
}