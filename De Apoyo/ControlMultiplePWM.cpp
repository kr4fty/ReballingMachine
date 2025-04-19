/*
 *  Implementación de un control con salidas multiples para el control de TRIAC
 *
 *  USO:
 *      Con el botón vamos seleccionando las salidas disponibles.
 *      Moviendo el encoder cambiamos el angulo de disparo del Triac
 *      seleccionado.
 * 
 *  Autor: Tapia Velasquez Favio
 */


#include <Arduino.h>
#include <esp_system.h>
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
uint16_t encoderValue, oldEncoderValue[MAX_OUTPUTS];
uint8_t selected;
uint8_t key;

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
  
  // Inicializo el Timer que realiza el pulso que activa el Triac 
  initPwmPulseSettings();

  setPwmPulse(LOWER_HEATER, encoderValue);
  setPwmPulse(UPPER_HEATER, encoderValue);
  setPwmPulse(EXTRACTOR, encoderValue);

  selected = 0;

  encoder_setBasicParameters(0, 180, false, oldEncoderValue[selected], 150);


  encoderValue = encoder_read();

  lcd.fillScreen(ST7735_BLACK);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
  lcd.setCursor(1*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
  lcd.print("PWM control");

  lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
  lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
  lcd.printf("  L HEATER");

  lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
  lcd.setCursor(1*6*BIG_TEXT, 2*8*BIG_TEXT+2);
  lcd.setTextSize(BIG_TEXT);
  lcd.printf("%4d", encoderValue);
  
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
    encoderValue = encoder_read();
    if (oldEncoderValue[selected] != encoderValue) {
      lcd.setCursor(1*6*BIG_TEXT, 2*8*BIG_TEXT+2);
      lcd.setTextSize(BIG_TEXT);
      lcd.printf("%4d", encoderValue);
      oldEncoderValue[selected] = encoderValue;
      setPwmPulse(selected, encoderValue);
    }
  }

  key = isButtonClicked();
  if(key){
    if(key == SHORT_CLICK){
      selected ++;
      if(selected >= MAX_OUTPUTS){
        selected = 0;
      }

      encoder_setBasicParameters(0, 180, false, oldEncoderValue[selected], 150);
      encoderValue = oldEncoderValue[selected];

      lcd.setCursor(1*6*BIG_TEXT, 2*8*BIG_TEXT+2);
      lcd.setTextSize(BIG_TEXT);
      lcd.printf("%4d", encoderValue);

      lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
      lcd.setTextSize(MIDLE_TEXT);
      lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
      switch(selected){
        case LOWER_HEATER:
          lcd.printf("  L HEATER");
          break;
        case UPPER_HEATER:
          lcd.printf("  U HEATER");
          break;
        case EXTRACTOR:
          lcd.printf(" EXTRACTOR");
          break;
        default:
          break;
      }
      lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);

    }
  }

  // basic readout test, just print the current temp
  actualTime = millis();
  if (actualTime > nextTime){ 
    tiempo = (actualTime-startTime)/1000.0;
    
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setCursor(0*6*LITTLE_TEXT, 14*8*LITTLE_TEXT);
    lcd.print(temperatura);

    #ifdef SERIAL_PLOTTER
    Serial.printf("$%d %d;",(uint16_t)tiempo, (uint16_t)temperatura);
    #endif
    
    nextTime = actualTime + WINDOW_1Seg;
    
  }
}