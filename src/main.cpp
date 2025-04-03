/******************************************************************************
*
*     Reballing Machine
*
*     Autor: Tapia Velasquez Favio
*     Version: 0.09b
*                   Primer intento de seguimiento de un Perfil de Temperatura
*
*
******************************************************************************/

#include <Arduino.h>
#include "config.h"
#include <PID_v1.h>
#include "encoder.h"
#include "ota_upgrade.h"
#include "pulse_control.h"
#include "thermocouples.h"

#define WindowSize 200
unsigned long windowStartTime;
unsigned long       startTime;
unsigned long        nextTime;

bool isPowerOn;

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
PID myPID = PID(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);

float perfilRamp;
uint16_t tiempo;
uint64_t time34Init;
uint8_t etapa=1;
 
// the setup function runs once when you press reset or power the board
void setup() {
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUTTON, INPUT_PULLUP);
    pinMode(ZC_PIN, INPUT);

    digitalWrite(RELAY_PIN,LOW);

    //Setpoint1 = 120;
    //turn the PID on
    myPID.SetSampleTime(WindowSize);
    myPID.SetOutputLimits(0, ZC_MAX_ANGLE);
    myPID.SetMode(AUTOMATIC);

    // Use this initializer if using a 1.8" TFT screen:
    initDisplay();

    // Encoder
    encoder_init();

    /********************************* OTA ***********************************/
    /* Se actualizará solo si el sistema arranca con el botón presionado      */
    #ifdef ARDUINO_OTA
    if(isButtonDown()){
        startOtaUpgrade();
    }
    #endif
    /****************************** END OTA **********************************/

    printPresentation();

    windowStartTime = millis();
    // Espero por pulsación de tecla para comenzar
    do{ 
        if (millis() > windowStartTime){
            readThermocouples(&Input1, &Input2);

            windowStartTime = millis() + WindowSize;
        }
    }while(!isButtonClicked());

    printLavels();

    initPwmPulseSettings();

    nextTime=millis() + WINDOW_1Seg;

    startTime = millis();

    perfil_temp[0] = Input1;
    perfil_time[0] = startTime/1000;

    t1=perfil_time[0]+perfil_time[1];  // 120
    t2=t1+perfil_time[2];              // 120+60=180
    t3=t2+perfil_time[3];              // 120+60+15=195
    t4=t3+perfil_time[4];              // 120+60+15+15=210
    t5=t4+perfil_time[5];              // 120+60+15+15+150=360

    rampt0t1=((perfil_temp[1]-perfil_temp[0])/perfil_time[1]);
    rampt1t2=((perfil_temp[2]-perfil_temp[1])/perfil_time[2]);
    rampt2t3=((perfil_temp[3]-perfil_temp[2])/perfil_time[3]);
    rampt3t4=((perfil_temp[4]-perfil_temp[3])/perfil_time[4]);
    rampt4t5=((perfil_temp[5]-perfil_temp[4])/perfil_time[5]);

    /*Serial.printf("%d %d %d %d %d\n",t1,t2,t3,t4,t5);
    Serial.printf("%.2f %.2f %.2f %.2f %.2f\n", rampt0t1, rampt1t2, rampt2t3, rampt3t4, rampt4t5);*/

    windowStartTime = millis();
    isPowerOn = true;
    printSystemStatus(isPowerOn);

    //DEBUG
    #ifdef DEBUG
    Serial.begin(9600);
    startTime = windowStartTime;
    #endif
}

// the loop function runs over and over again forever
void loop() {
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

    /****************************** START/STOP *******************************/
    if(isButtonClicked()) {
        isPowerOn = not isPowerOn;
        printTicks(Output1);

        printSystemStatus(isPowerOn);

        if(!isPowerOn) {
            myPID.SetMode(MANUAL);
            stopZcInterrupt(); // Desactivo interrupciones de cruce por cero
        }
        else {
            myPID.SetMode(AUTOMATIC);
            startZcInterrupt(); // Activo interrupciones de cruce por cero
        }
    }
    /*************************************************************************/

    if(isPowerOn){
    /********************************** PID **********************************/
        // Toma Input y actúa. En la variable Output se guarda lo calculado
        myPID.Compute();

    /*************************************************************************/

    /***************************** CONTROL DE FASE ***************************/
        // Se envía pulso de habilitación del TRIAC
        setPwmPulse(Output1);

        #ifdef ZC_INTERRUPT_FILTER
        // Filtro para detectar falsos Cruces por Cero
        if(millis()>zcNextTime && zcFlag){
            zcFlag = false;
            zcCounter++;
        }
        #endif
    /*************************************************************************/
    }

    if(millis()>nextTime){  // Una vez por segundo
        /****** SOLO PARA PRUEBAS!!!! ********/
        startTime = millis();
        /*************************************/

        // Perfil térmico *****************************************************
        tiempo =(uint16_t)((millis()-startTime)/1000);
        // Trazado del perfil ideal
        if     (tiempo < t1)
            perfilRamp = rampt0t1*(tiempo-perfil_time[0])+perfil_temp[0];
        else if(tiempo < t2)
            perfilRamp = rampt1t2*(tiempo-t1)+perfil_temp[1];
        else if(tiempo < t3)
            perfilRamp = rampt2t3*(tiempo-t2)+perfil_temp[2];
        else if(tiempo < t4)
            perfilRamp = rampt3t4*(tiempo-t3)+perfil_temp[3];
        else if(tiempo < t5)
            perfilRamp = rampt4t5*(tiempo-t4)+perfil_temp[4];
        else
            perfilRamp = 0;

        switch (etapa)
        {
            case 1: // precalentamiento
                    //myPID.SetTunings(0.5, 0.0038989, 9.2996);
                    myPID.SetTunings(0.2950, 0.002774, 6.0934);
                    //myPID.SetTunings(0.250, 0.001949, 4.19498);
                    Setpoint1 = perfil_temp[1]+75;
                    if(Input1>=(perfil_temp[1]-10)){
                        time34Init = millis();
                        etapa = 2;
                    }
                    break;
            case 2: // activación de flux
                    //myPID.SetTunings(1, 0.007798, 18.5991);
                    //myPID.SetTunings(0.5, 0.0038989, 9.2996);
                    //myPID.SetTunings(0.2950, 0.0023, 5.4877);
                    myPID.SetTunings(0.250, 0.001949, 4.19498);
                    Setpoint1 = perfil_temp[2]+30;
                    if((Input1>=perfil_temp[2]) && (millis()>(time34Init+perfil_time[2]*1000))){
                        time34Init = millis();
                        etapa = 3;
                    }
                    break;
            case 3: // reflow
                    //myPID.SetTunings(1, 0.007798, 18.5991);
                    //myPID.SetTunings(0.5, 0.0038989, 9.2996);
                    myPID.SetTunings(0.2950, 0.0023, 5.4877);
                    Setpoint1 = perfil_temp[3]+105;
                    if((Input1>=(perfil_temp[3]-30)) && (millis()>(time34Init+perfil_time[3]*1000))){
                        time34Init = millis();
                        etapa = 4;
                    }
                    break;
            case 4: // extracción
                    //myPID.SetTunings(1, 0.007798, 18.5991);
                    //myPID.SetTunings(0.5, 0.0038989, 9.2996);
                    myPID.SetTunings(0.2950, 0.0023, 5.4877);
                    Setpoint1 = perfil_temp[4];
                    if(millis()>(time34Init+perfil_time[4]*1000))
                        etapa = 5;
                    break;
            case 5: // enfriamiento
                    Setpoint1 = perfil_temp[5];
                    stopZcInterrupt();
                    break;
            default:
                    break;
        }
        //*********************************************************************

        if(oldInput1!=Input1 || oldInput2!=Input2){
            printInputs(Input1, Input2);

            oldInput1 = Input1;
            oldInput2 = Input2;
        }

        printOutputs(Output1, zcCounter);

        printActualSetpoint((uint16_t)perfil_temp[etapa]);

        /******* CHEQUEO DE CORRESPONDENCIA PULSOS ENVIADOS/RECIBIDOS ********/
        printPulsesStatus((abs(zcCounter-totalPulsesSent)<=2));
        totalPulsesSent = 0;
        /*********************************************************************/

        // DEBUG
        #ifdef DEBUG
        //Time = (millis()-startTime)/1000.0;
        Serial.printf("$%.2f %.2f %d %.2f;",Input1, Input2, etapa, perfilRamp);
        #endif

        zcCounter=0;
        nextTime=millis() + WINDOW_1Seg;      
    }
}
