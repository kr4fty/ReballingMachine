#include <Arduino.h>
#include "config.h"
#include <SPI.h>
#include <Wire.h>
#include <max6675.h>
#include <ESP32Encoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define WindowSize 200
unsigned long windowStartTime, startTime;

ESP32Encoder encoder;
Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
MAX6675 thermocouple(TH1_CS);

double Time;
long int actualDutty=230, oldDutty=0, actualFrec=PWM1_Freq, oldFrec=0;

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN,OUTPUT);
  
  digitalWrite(RELAY_PIN,HIGH);

  Serial.begin(115200);

  lcd.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Para sobreescribir puntos
  lcd.fillScreen(ST77XX_BLACK);
  lcd.cp437(true);
  lcd.setRotation(1);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.print("Press twice");

  while(digitalRead(BUTTON));
    while(!digitalRead(BUTTON));

  delay(200);

  while(digitalRead(BUTTON));
    while(!digitalRead(BUTTON));

  
  ledcAttachPin(RELAY_PIN, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  
  digitalWrite(LED_BUILTIN, HIGH);
  windowStartTime = millis();
  startTime = windowStartTime;

  ESP32Encoder::useInternalWeakPullResistors = UP;
  //encoder.attachSingleEdge(25,26);
  encoder.attachHalfQuad(25, 26);
  //encoder.attachFullQuad(25, 26);
  encoder.clearCount();
  encoder.setFilter(1023);
  encoder.setCount(actualDutty);

  lcd.fillScreen(ST77XX_BLACK);
  lcd.setCursor(0, 0);
  lcd.println(" Dutty: ");
  lcd.println(" Frec: ");
  lcd.setTextSize(MIDLE_TEXT);
}

uint8_t actualCursor=0, oldCursor;
void loop() {
  
  if(!digitalRead(BUTTON)){
    delay(100);
    if((oldCursor+1)>1){
      oldCursor = actualCursor;
      actualCursor = 0;
      encoder.setCount(actualDutty);
    }
    else{
      oldCursor = actualCursor;
      actualCursor = 1;
      encoder.setCount(actualFrec);
    }

    lcd.setCursor(0, oldCursor*8*MIDLE_TEXT);
    lcd.print(" ");
    lcd.setCursor(0, actualCursor*8*MIDLE_TEXT);
    lcd.print(">");
  }

  
  if(actualCursor == 0){
    actualDutty = encoder.getCount();
    if(actualDutty<0 || actualDutty>Resolution){
      if(actualDutty<0)
        actualDutty = Resolution;
      else
        actualDutty = 0;
      encoder.setCount(actualDutty);
    }
    if (oldDutty != actualDutty) {
      lcd.setCursor(7*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
      lcd.printf("%4d", actualDutty);
      oldDutty = actualDutty;
      ledcWrite(PWM1_Ch, actualDutty);
    }
  }
  else if(actualCursor == 1){
    actualFrec = encoder.getCount();
    if(actualFrec<0 || actualFrec>1000){
      if(actualFrec<0)
        actualFrec = 1000;
      else
        actualFrec = 0;
      encoder.setCount(actualFrec);
    }
    if (oldFrec != actualFrec) {
      lcd.setCursor(7*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
      lcd.printf("%4d", actualFrec);
      oldFrec = actualFrec;
      ledcChangeFrequency(PWM1_Ch, actualFrec, PWM1_Res);
    }
  }

  // basic readout test, just print the current temp
  if (millis() > windowStartTime){ 
    Time = (millis()-startTime)/1000.0;
    Serial.print(thermocouple.readCelsius());
    Serial.print(" ");
    Serial.println(Time);
    windowStartTime = millis() + WindowSize;
    
  }
}