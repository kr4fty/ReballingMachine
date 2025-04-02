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
long int encoderCounter=230, oldEncoderCounter=0;

uint32_t zcNextTime=0, nextTime; 
uint64_t pulseOn, pulseOff, shootingAngle=0;
bool zcFlag=false, pulseFlag=false, pulseStatus;
long int zcCounter=0;

void IRAM_ATTR zc_isr() {
  if(!zcFlag){
    zcFlag = true;
    zcNextTime=millis()+WINDOW_INTERRUPT;
    pulseFlag = true;
    pulseOn = micros()+shootingAngle;
  }
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(ZC_PIN, INPUT);
  
  digitalWrite(RELAY_PIN,HIGH);

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

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

  /*// Se reemplaza PWM digital por control de cruce por cero y angulo de disparo
  ledcAttachPin(RELAY_PIN, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);*/
  
  digitalWrite(LED_BUILTIN, HIGH);
  windowStartTime = millis();
  startTime = windowStartTime;

  ESP32Encoder::useInternalWeakPullResistors = UP;
  //encoder.attachSingleEdge(25,26);
  encoder.attachHalfQuad(25, 26);
  //encoder.attachFullQuad(25, 26);
  encoder.clearCount();
  encoder.setFilter(1023);
  encoder.setCount(encoderCounter);

  lcd.fillScreen(ST77XX_BLACK);
  lcd.setCursor(0*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
  lcd.print("Dutty: ");

  attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_isr,HIGH);
  nextTime=millis() + WINDOW_1Seg;
}

void loop() {
  encoderCounter = encoder.getCount(); // Para el control de fase, seteo manual


  if(millis()>zcNextTime && zcFlag){
    zcFlag = false;
    zcCounter++;
  }
  if(micros()>pulseOn && pulseFlag){
    pulseOff = micros() + G_PULSE_WIDTH ;
    pulseStatus = HIGH;
    digitalWrite(RELAY_PIN, pulseStatus);
    pulseFlag = false;
  }
  if(micros()>pulseOff && !pulseFlag && pulseStatus){
    pulseStatus = LOW;
    digitalWrite(RELAY_PIN, pulseStatus);    
  }

  if(encoderCounter<0 || encoderCounter>ZC_MAX_ANGLE){
    if(encoderCounter<0)
      encoderCounter = ZC_MAX_ANGLE;
    else
      encoderCounter = 0;
    encoder.setCount(encoderCounter);
  }
  if (oldEncoderCounter != encoderCounter) {
    lcd.setCursor(1*6*BIG_TEXT, 1*8*BIG_TEXT+2);
    lcd.setTextSize(BIG_TEXT);
    lcd.printf("%4d", encoderCounter);
    oldEncoderCounter = encoderCounter;
    //ledcWrite(PWM1_Ch, encoderCounter);
    shootingAngle = map(encoderCounter, 0, ZC_MAX_ANGLE, 0, 10000);
    lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.printf("%5d", shootingAngle);
  }
  
  #ifdef DEBUG
  // basic readout test, just print the current temp
  if (millis() > windowStartTime){ 
    Time = (millis()-startTime)/1000.0;

    Serial.print(thermocouple.readCelsius());
    Serial.print(" ");
    Serial.println(Time);
    
    windowStartTime = millis() + WindowSize;
    
  }
  #endif

    // cuento interrupsiones de cruce por cero, por segundo
  if(millis()>nextTime){
    #ifdef DEBUG
    Serial.println(counter);
    #endif

    lcd.setCursor(6*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.printf("%3d", zcCounter);
    zcCounter=0;
    nextTime=millis() + WINDOW_1Seg;
  }
}