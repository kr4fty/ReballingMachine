/******************************************************************************
*
*     Reballing Machine
*
*     Autor: Tapia Velasquez Favio
*     Version: 0.08f
*
*
******************************************************************************/

#include <Arduino.h>
#include <PID_v1.h>
#include "config.h"
#include "pulse_control.h"
#include "display.h"
#include "encoder.h"
#include "thermocouples.h"
#ifdef ARDUINO_OTA
#include "ota_upgrade.h"
#endif


#define WindowSize 200
unsigned long windowStartTime;
uint32_t      nextTime;

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
/* Se actulizará solo si el sistema arranca con el boton presionado          */
  #ifdef ARDUINO_OTA
  if(digitalRead(BUTTON)==LOW){
    startOtaUpgrade();
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

  // Imprimo en el display los titulos de cada variable a mostrar
  printLavels();
  // Inicializo Timers encargados de generar el pulso de activacion del triac
  initPwmPulseSettings();
  
  oldEncoderCounter=-1;   // 
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
  // Mostrar en display en caso de haber cambios en el setpoint
  if (oldEncoderCounter != encoderCounter) {
    printEncoderValue(encoderCounter);
    Setpoint1 = encoderCounter;
    oldEncoderCounter = encoderCounter;
  }

  // Senso la temperatura cada 200mSeg (frec. máxima que lee el sensor max6675)
  if (millis() > windowStartTime){
    // Leo las temperaturas en los calentadores
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
    // Dentro de WindowSize (200mSeg), vuelvo a sensar las nuevas temperaturas
    windowStartTime = millis() + WindowSize;
  }
  // Calculo del valor de salida, PID, segun el Input leido
  myPID.Compute();

/****************************** CONTROL DE FASE ******************************/
  // En base a la Salida calculada configuro el Angulo de Disparo del pulso PWM
  setPwmPulse(Output1);

  #ifdef ZC_INTERRUPT_FILTER
  // Filtro para detectar falsos Cruces por Cero
  if(millis()>zcNextTime && zcFlag){
    zcFlag = false;
    zcCounter++;
  }
  #endif
/**************************** END CONTROL DE FASE ****************************/
  // Por cada seg se imprimen los valores de temperatura actual y otras salidas
  if(millis()>nextTime){
    // Solo se muestra si alguana de las temperaturas cambiaron
    if(oldInput1!=Input1 || oldInput2!=Input2){
      printInputs(Input1, Input2);
      oldInput1 = Input1;
      oldInput2 = Input2;
    }
    printOutputs(Output1, zcCounter);
    // Reset de la cantidad de deteccin de Cruce por Cero, por segundo
    zcCounter=0;

    // DEBUG
    #ifdef DEBUG
    Serial.printf("$%.2f %.2f %d;",Input1, Input2, (uint16_t)Output1);
    #endif
    // Vuelvo a imprimir dentro de 1 segundo
    nextTime=millis() + WINDOW_1Seg;      
  }
}