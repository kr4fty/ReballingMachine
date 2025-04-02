/***************************************************************************
  sTune PID_v1 Example (MAX6675, PTC Heater / SSR / Software PWM)
  This sketch does on-the-fly tuning and PID control. Tuning parameters are
  quickly determined and applied during temperature ramp-up to setpoint.
  View results using serial plotter.
  Reference: https://github.com/Dlloydev/sTune/wiki/Examples_MAX6675_PTC_SSR
  ***************************************************************************/
#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <max6675.h>
#include <sTune.h>
#include <PID_v1.h>


// user settings
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 170;  // runPid interval = testTimeSec / samples
const uint16_t samples = 1000;
const float inputSpan = 450;
const float outputSpan = 180;
float outputStart = 0;
float outputStep = 180;
float tempLimit = 400;
uint8_t debounce = 1;

// variables
double input, output, setpoint = 200, kp, ki, kd; // PID_v1
float Input, Output, Setpoint = 200, Kp, Ki, Kd; // sTune
float Temp;

MAX6675 thermocouple1(TH1_CS); //HW SPI
MAX6675 thermocouple2(TH2_CS); //HW SPI

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);
PID myPID(&input, &output, &setpoint, kp, ki, kd, P_ON_M, DIRECT);

uint32_t zcNextTime=0, nextTime; 
uint64_t pulseOn, pulseOff, pulseDelay=0;
bool zcFlag=false, pulseFlag=false, pulseStatus;
long int zcCounter=0;
#define WindowSize 200
unsigned long windowStartTempTime;

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
float softPwm(const uint8_t relayPin, float input, float output, float setpoint, uint32_t windowSize, uint8_t debounce);

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  Serial.begin(9600);
  delay(3000);
  Output = 0;
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);

  windowStartTempTime = millis();
  attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_isr,RISING);
}
float optimumOutput;
void loop() {
  optimumOutput = softPwm(RELAY_PIN, Input, Output, Setpoint, outputSpan, debounce);
  if (millis() > windowStartTempTime){
    Temp = thermocouple1.readCelsius();
    thermocouple2.readCelsius();
    #ifdef ALPHA
    InputFiltered = (ALPHA*Temp) + ((1-ALPHA)*InputFiltered);
    Temp = InputFiltered;
    #endif
    windowStartTempTime = millis() + WindowSize;
  }

  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      Input=Temp;
      tuner.plotter(Input, Output, Setpoint, 1, 3); // output scale 0.5, plot every 3rd sample
      //Serial.printf("$%d %.2f %d;", (uint16_t)Setpoint, Input,  (uint16_t)Output);
      
      break;

    case tuner.tunings: // active just once when sTune is done
      /*tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
      myPID.SetOutputLimits(0, outputSpan);
      myPID.SetSampleTime(WindowSize);
      debounce = 0; // ssr mode
      setpoint = Setpoint, output = outputStep, kp = Kp, ki = Ki, kd = Kd;
      Output = outputStep;
      myPID.SetMode(AUTOMATIC); // the PID is turned on
      myPID.SetTunings(kp, ki, kd); // update PID with the new tunings
      break;*/
      Output = 0;
      tuner.SetTuningMethod(tuner.TuningMethod::Mixed_PID);
      tuner.printTunings();
      tuner.printResults();
      break;
    /*case tuner.runPid: // active once per sample after tunings
      Input = Temp;
      input = Input;
      myPID.Compute();
      Output = output;
      tuner.plotter(Input, optimumOutput, Setpoint, 1, 3);
      tuner.printResults();
      //Serial.printf("$%.2f %.2f %.2f;", Input, optimumOutput, Setpoint);
      break;*/
  }
}

float softPwm(const uint8_t relayPin, float input, float output, float setpoint, uint32_t windowSize, uint8_t debounce) {

  // software PWM timer
  uint32_t msNow = millis();
  static uint32_t  windowStartTime, nextSwitchTime;
  if (msNow - windowStartTime >= windowSize) {
    windowStartTime = msNow;
  }
  // SSR optimum AC half-cycle controller
  static float optimumOutput;
  static bool reachedSetpoint;

  if (input > setpoint) reachedSetpoint = true;
  if (reachedSetpoint && !debounce && setpoint > 0 && input > setpoint) optimumOutput = output - 8;
  else if (reachedSetpoint && !debounce && setpoint > 0 && input < setpoint) optimumOutput = output + 8;
  else  optimumOutput = output;
  if (optimumOutput < 0) optimumOutput = 0;

  // PWM relay output
  /***************************** CONTROL DE FASE ****************************/
  pulseDelay = t10mSEG-map((uint8_t)output, 0, ZC_MAX_ANGLE, G_PULSE_WIDTH, t10mSEG-ZC_PULSE_WIDTH/2);

  // Filtro para falsos positivos
  #ifdef ZC_INTERRUPT_FILTER
  if(millis()>zcNextTime && zcFlag){
    zcFlag = false;
  }
  #endif

  // Se envia pulso de habilitacion del TRIAC
  if(micros()>pulseOn && pulseFlag){
    pulseOff = micros() + G_PULSE_WIDTH ;
    pulseStatus = HIGH;
    pulseFlag = false;
    if(pulseDelay<(t10mSEG-G_PULSE_WIDTH))    // Envia pulso solo si Output >0
      digitalWrite(RELAY_PIN, pulseStatus);

  }
  if(micros()>pulseOff && !pulseFlag && pulseStatus){
    pulseStatus = LOW;
    if(pulseDelay>ZC_PULSE_WIDTH/2)
      digitalWrite(RELAY_PIN, pulseStatus);   // Baja pulso solo si Output <180
  }
  /*************************************************************************/
  return optimumOutput;
}