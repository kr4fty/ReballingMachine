#include <Arduino.h>
#include <PID_v1.h>
#include "config.h"
#include "thermocouples.h"
#include "encoder.h"
#include "display.h"
#include "pulse_control.h"
#include "my_profiles.h"

#define LED_BUILTIN 2

#define WindowSize 200
#define WindowSize_cooler 200
#define SerialPing  2000
unsigned long nextTime, startTime, actualTime, windowStartTime, lastMessage=0; 

double tiempo;
double temperatura, tAmb=24;

long int encoderCounter=0, oldEncoderCounter=0;

double input, output, setpoint;
double kp=KP_COOLER, ki=KI_COOLER, kd=KD_COOLER;
PID myCoolerPID = PID(&input, &output, &setpoint, kp, ki, kd, REVERSE);

float perfilRamp;
uint8_t etapa=1;

double offset;

bool isPowerOn;

uint8_t key;

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(ZC_PIN, INPUT);

  digitalWrite(LED_BUILTIN,LOW);

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  encoder_init();
  initDisplay();
  // Inicializo los Timers que realizaran el pulso que activa el Triac 
  initPwmPulseSettings();
  setPwmPulse(0);

  // PID
  myCoolerPID.SetSampleTime(WindowSize_cooler);
  myCoolerPID.SetOutputLimits(0, 1); // para el angulo de disparo es 180-angulo
  myCoolerPID.SetMode(AUTOMATIC);
  setpoint = 40;

  lcd.setTextSize(MIDLE_TEXT);
  lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
  lcd.print("Iniciando...");
  lcd.setTextSize(LITTLE_TEXT);

  Serial.print("Esperando por Parámetros PID\n");
  Serial.print("Ingrese separados por un espacio y sin ENTER\n");
  Serial.print("Formato:\n");
  Serial.print("         Kp.pp Ki.ii Kd.dd\n");

  encoder_setBasicParameters(0, 180, false, 90, 150);
  nextTime = actualTime + 200;
  while(!isButtonClicked()){
    if (actualTime > nextTime){ 
      readThermocouple(&temperatura);
      input = temperatura;

      lcd.setTextSize(LITTLE_TEXT);
      lcd.setCursor(0*6*LITTLE_TEXT, 10*8*LITTLE_TEXT);
      lcd.print(temperatura);
      
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

    #ifdef DEBUG
    actualTime = millis();
    if((actualTime - lastMessage) > SerialPing) {  //If its been long enough give us some info on serial
      if (Serial.available() > 0) { //If we sent the program a command deal with it
        for (int x = 0; x < 4; x++) {
          switch (x) {
            case 0:
              kp = Serial.parseFloat();  
              break;
            case 1:
              ki = Serial.parseFloat();
              break;
            case 2:
              kd = Serial.parseFloat();
              break;
            case 3:
              for (int y = Serial.available(); y == 0; y--) {
                Serial.read();  //Clear out any residual junk
              }
              break;
          }
        }
        Serial.printf("%.3f %.3f %.3f\n\t", kp, ki, kd);
        myCoolerPID.SetTunings(kp, ki, kd); //Set the PID gain constants and start running
      }
      
      lastMessage = actualTime; 
      //update the time stamp. 
    }
    #endif
    actualTime = millis();
  }
  
  Serial.printf("kp: %.3f - ki: %.3f - kd: %.3f\n", kp, ki, kd);

  profile_initializeProfiles();
  profile_selectProfile(SN60PB40v1);
  
  digitalWrite(LED_BUILTIN, HIGH);
  isPowerOn = true;

  lcd.fillScreen(ST7735_BLACK);
  lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
  lcd.setCursor(1*6*BIG_TEXT, 1*8*BIG_TEXT+2);
  lcd.setTextSize(BIG_TEXT);
  lcd.printf("%4d", encoderCounter);

  lcd.setTextSize(LITTLE_TEXT);
  lcd.setCursor(0*6*LITTLE_TEXT, 0*8*LITTLE_TEXT);
  lcd.println("Time:");
  lcd.println("Temp:");
  lcd.println("Spnt:");
  lcd.println("Oupt:");
  lcd.print("Etapa:");

  lcd.setCursor(15*6*LITTLE_TEXT, 0*8*LITTLE_TEXT);
  lcd.printf("Kp: %2.5f", kp);
  lcd.setCursor(15*6*LITTLE_TEXT, 1*8*LITTLE_TEXT);
  lcd.printf("Ki: %2.5f", ki);
  lcd.setCursor(15*6*LITTLE_TEXT, 2*8*LITTLE_TEXT);
  lcd.printf("Kd: %2.5f", kd);

  drawChart();
  
  startTime = millis();
  windowStartTime = startTime;
  nextTime = startTime + WINDOW_1Seg;

  profile_preCalculate(input, (uint16_t)((millis()-startTime)/1000));

  // Para saltar a la etapa que deseamos
  //etapa = myProfile.length - 1;
  //offset = myProfile.time[myProfile.length-2];
  //Serial.printf("Offset: %4d\n", (unsigned long)offset);
}

void loop() {
  // Senso la temperatura cada 200mSeg 
  // (frecuencia máxima a la que lee el sensor max6675)
  actualTime = millis();
  if (actualTime > windowStartTime){
    readThermocouple(&temperatura);

    input = temperatura;

    // Perfil térmico *****************************************************
    tiempo = (actualTime-startTime)/1000.0;
    tiempo += offset; // Para saltar a la etapa que deseamos

    // Trazado del perfil ideal
    if(tiempo < myProfile.time[etapa] && etapa < myProfile.length){
        perfilRamp = tempSlope[etapa-1]*(tiempo-myProfile.time[etapa-1]) + myProfile.temperature[etapa-1];
        if((uint16_t)(tiempo+1)==myProfile.time[etapa]){
            // Paso a la siguiente etapa
            etapa++;
        }

    }
    else{
        perfilRamp = tAmb;
    }
    //setpoint = perfilRamp;

    windowStartTime = actualTime + WindowSize;
  }

  if(isPowerOn){ // En funcionamiento?
    myCoolerPID.Compute();
    // Se envía pulso de habilitación del TRIAC
    if(output==COOLER_MIN_ANGLE){
      output = 0;
    }
    else if(output==COOLER_MAX_ANGLE){
      output = ZC_MAX_ANGLE;
    }
    //setPwmPulse(output);
    // MOC3041
    if(!output){
      digitalWrite(RELAY_PIN, LOW);
    }
    else{
      digitalWrite(RELAY_PIN, HIGH);     // pequeño pulso para tener tiempo de activar el MOC3041
    }
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

  if(key=isButtonClicked()) {
    if(key == SHORT_CLICK){
      isPowerOn = not isPowerOn;

      digitalWrite(LED_BUILTIN, isPowerOn);

      if(!isPowerOn) {
        myCoolerPID.SetMode(MANUAL);
        stopZcInterrupt(); // Desactivo interrupciones de cruce por cero
      }
      else {
        myCoolerPID.SetMode(AUTOMATIC);
        startZcInterrupt(); // Activo interrupciones de cruce por cero
      }
    }
    else if(key == LONG_CLICK){
      Serial.print("Reiniciando MCU...\n");
      esp_restart();
    }
  }

  // basic readout test, just print the current temp
  actualTime = millis();
  if (actualTime > nextTime){
    tiempo = (actualTime-startTime)/1000.0;

    lcd.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setCursor(6*6*LITTLE_TEXT, 0*8*LITTLE_TEXT);
    lcd.printf("%3d",(unsigned long)tiempo);
    lcd.setCursor(6*6*LITTLE_TEXT, 1*8*LITTLE_TEXT);
    lcd.printf("%3d",(uint16_t)input);
    lcd.setCursor(6*6*LITTLE_TEXT, 2*8*LITTLE_TEXT);
    lcd.printf("%3d", (uint16_t)setpoint);
    lcd.setCursor(6*6*LITTLE_TEXT, 3*8*LITTLE_TEXT);
    lcd.printf("%3d", (uint16_t)output);
    lcd.setCursor(6*6*LITTLE_TEXT, 4*8*LITTLE_TEXT);
    lcd.printf(" %2d", etapa);

    printPoint(tiempo, (uint16_t)input, ST7735_YELLOW);
    printPoint(tiempo, (uint16_t)(output-COOLER_MIN_ANGLE), ST7735_CYAN, 3, 2);

    #ifdef DEBUG
    actualTime = millis();
    if((actualTime - lastMessage) > SerialPing) {  //If its been long enough give us some info on serial
      if (Serial.available() > 0) { //If we sent the program a command deal with it
        for (int x = 0; x < 4; x++) {
          switch (x) {
            case 0:
              kp = Serial.parseFloat();  
              break;
            case 1:
              ki = Serial.parseFloat();
              break;
            case 2:
              kd = Serial.parseFloat();
              break;
            case 3:
              for (int y = Serial.available(); y == 0; y--) {
                Serial.read();  //Clear out any residual junk
              }
              break;
          }
        }
        //Serial.printf("%.3f %.3f %.3f\n\t", kp, ki, kd);
        lcd.setCursor(15*6*LITTLE_TEXT, 0*8*LITTLE_TEXT);
        lcd.printf("Kp: %2.5f", kp);
        lcd.setCursor(15*6*LITTLE_TEXT, 1*8*LITTLE_TEXT);
        lcd.printf("Ki: %2.5f", ki);
        lcd.setCursor(15*6*LITTLE_TEXT, 2*8*LITTLE_TEXT);
        lcd.printf("Kd: %2.5f", kd);

        myCoolerPID.SetTunings(kp, ki, kd); //Set the PID gain constants and start running
      }
      
      lastMessage = actualTime; 
      //update the time stamp. 
    }
    #endif

    #ifdef DEBUG
    //Serial.printf("$%d %d;",(uint16_t)setpoint, (uint16_t)temperatura);
    //Serial.printf("Time:%3d\nTemp:%3d\nSpnt:%3d\nOupt:%3d\nEtapa: %2d\n\n",(unsigned long)tiempo, (uint16_t)input, (uint16_t)setpoint, (uint16_t)output, etapa);
    #endif
    
    nextTime = actualTime + WINDOW_1Seg;
  }
}