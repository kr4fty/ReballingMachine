/******************************************************************************
*
*     Reballing Machine
*
*     Autor: Tapia Velasquez Favio
*     Version: 0.08e
*
*
******************************************************************************/

#include <Arduino.h>
#include <PID_v1.h>
#include "config.h"
#include "pulse_control.h"
#ifdef ARDUINO_OTA
#include "ota_upgrade.h"
#endif
#include "display.h"
#include "encoder.h"
#include "thermocouples.h"

uint32_t nextTime;

#define WindowSize 200
unsigned long windowStartTime;
//DEBUG
#ifdef DEBUG
unsigned long startTime;
#endif

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
PID myPID = PID(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
 
// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(ZC_PIN, INPUT);

  digitalWrite(RELAY_PIN,LOW);

  initDisplay();

/********************************** OTA **************************************/
/* Se actulizara solo si el sistema arranca con el boton presionado          */
  #ifdef ARDUINO_OTA
  if(digitalRead(BUTTON)==LOW){
    startOtaUpgrade(lcd);
  }
  #endif
/******************************** END OTA ************************************/

  //Setpoint1 = 120;
  //turn the PID on
  myPID.SetSampleTime(WindowSize);
  myPID.SetOutputLimits(0, ZC_MAX_ANGLE);
  myPID.SetMode(AUTOMATIC);

  printLavelsInit();

  // Encoder
  initEncoder();

  windowStartTime = millis();
  do{
    readEncoder();

    if (oldEncoderCounter != encoderCounter) {
      printEncoderValue(encoderCounter);

      oldEncoderCounter = encoderCounter;
      //pulseDelay = t10mSEG-map(encoderCounter, 0, ZC_MAX_ANGLE, G_PULSE_WIDTH, t10mSEG-ZC_PULSE_WIDTH/2);
    }
    if (millis() > windowStartTime){
      readThermocouples(&Input1, &Input2);

      windowStartTime = millis() + WindowSize;
    }
  }while(!isButtonClicked());

  printLavels();

  initPwmPulseSettings();
  
  oldEncoderCounter=-1;
  nextTime=millis() + WINDOW_1Seg;
  windowStartTime = millis();
  //DEBUG
  #ifdef DEBUG
  //Serial.begin(9600);
  startTime = windowStartTime;
  #endif
}

// the loop function runs over and over again forever
void loop() {

  // Seteo del Setpoint
  readEncoder();
  if (oldEncoderCounter != encoderCounter) {
    printEncoderValue(encoderCounter);
    Setpoint1 = encoderCounter;
    oldEncoderCounter = encoderCounter;
  }

  //Senso la temperatura cada 200mSeg (frecuencia máxima a la que lee el sensor max6675)
  if (millis() > windowStartTime){
    readThermocouples(&Input1, &Input2);

    // DEBUG
    #ifdef DEBUG
    //double Time = (millis()-startTime)/1000.0;
    //Serial.print(Input1);
    //Serial.print(" ");
    //Serial.println(0);
    #ifdef ALPHA
    //Serial.printf("$%.2f %.2f %.2f;",Input1,InputFiltered1,Output1);
    #else
    //Serial.printf("$%.2f %.2f;",Input1,Output1);
    #endif
    #endif

    windowStartTime = millis() + WindowSize;
  }

  myPID.Compute();

/****************************** CONTROL DE FASE ******************************/
  setPwmPulse(Output1);

  #ifdef ZC_INTERRUPT_FILTER
  // Filtro para detectar falsos Cruces por Cero
  if(millis()>zcNextTime && zcFlag){
    zcFlag = false;
    zcCounter++;
  }
  #endif
/**************************** END CONTROL DE FASE ****************************/

  if(millis()>nextTime){

    if(oldInput1!=Input1 || oldInput2!=Input2){
      printInputs(Input1, Input2);
      oldInput1 = Input1;
      oldInput2 = Input2;
    }
    printOutputs(Output1, zcCounter);
    zcCounter=0;

    // DEBUG
    #ifdef DEBUG
    Serial.printf("$%.2f %.2f %d;",Input1, Input2, (uint16_t)Output1);
    #endif

    nextTime=millis() + WINDOW_1Seg;      
  }
}