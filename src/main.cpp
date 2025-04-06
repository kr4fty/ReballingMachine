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
#include "my_profiles.h"

#define WindowSize 200
unsigned long windowStartTime;
unsigned long       startTime;
unsigned long        nextTime;

bool isPowerOn;
unsigned long encoderValue;
unsigned long oldEncoderValue;

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
PID myPID = PID(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);

float perfilRamp;
uint16_t tiempo;
uint8_t etapa=1;
 
// the setup function runs once when you press reset or power the board
void setup() {
    //DEBUG
    //#ifdef DEBUG
    Serial.begin(115200);
    //#endif

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

    //printPresentation();

    /************************ SELECCIÓN DEL PERFIL A UTILIZAR ****************/
    // Busco los Perfiles disponibles e inicializo
    profile_initializeProfiles();
    printProfiles(profileNames, profileCount);
    encoder_setBasicParameters(0,profileCount-1, true, 0, 10);
    encoderValue = oldEncoderValue = myEnc.readEncoder();
    printProfileSelection(encoderValue+1);
    windowStartTime = millis();
    // Espero por pulsación de tecla para comenzar
    do{ 
        if (millis() > windowStartTime){
            readThermocouples(&Input1, &Input2);

            windowStartTime = millis() + WindowSize;
        }
        // Encoder
        if(myEnc.encoderRotationDetected()){
            // borro el anterior
            clearProfileSelection(oldEncoderValue+1);
            encoderValue = myEnc.readEncoder();
            printProfileSelection(encoderValue+1);
            oldEncoderValue = encoderValue;
        }
    }while(!isButtonClicked());
    profile_selectProfile(encoderValue);
    printSelectedProfile(profileNames[encoderValue], myProfile.time, myProfile.temperature, myProfile.length);
    /*************************************************************************/

    printLavels();

    // Inicializo los Timers que realizaran el pulso que activa el Triac 
    initPwmPulseSettings();

    // Selecciono el Perfil a utilizar;
    //profileSelectedIndex = SN63PB37;


    nextTime=millis() + WINDOW_1Seg;
    startTime = millis();

    // Realizo cálculos iniciales de acuerdo al Perfil seleccionado
    profile_preCalculate(Input1, (uint16_t)((millis()-startTime)/1000));

    /*Serial.println("\tTime\tTemp\tSlope");
    for(uint8_t i=0; i<myProfile.length; i++){
        Serial.printf("%d-\t%d\t%d\t%.2f\n", i, myProfile.time[i], myProfile.temperature[i], tempSlope[i]);
    }*/

    windowStartTime = millis();
    isPowerOn = true;
    printSystemStatus(isPowerOn);
}

// the loop function runs over and over again forever
void loop() {
    // Senso la temperatura cada 200mSeg 
    // (frecuencia máxima a la que lee el sensor max6675)
    if (millis() > windowStartTime){
        readThermocouples(&Input1, &Input2);
        //Serial.printf("$%.2f %.2f %.2f;",Input1, perfilRamp, Output1);

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

    if(isPowerOn){ // En funcionamiento?
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
        // Perfil térmico *****************************************************
        tiempo =(uint16_t)((millis()-startTime)/1000);

        // Trazado del perfil ideal
        if(tiempo < myProfile.time[etapa] && etapa < myProfile.length){
            perfilRamp = tempSlope[etapa-1]*(tiempo-myProfile.time[etapa-1]) + myProfile.temperature[etapa-1];
            //Serial.printf("%d - %d\n", (tiempo+1), myProfile.time[etapa]);
            if((tiempo+1)==myProfile.time[etapa]){
                // Paso a la siguiente etapa
                etapa++;
            }
        }
        else{
            perfilRamp = 0;
        }
        Setpoint1 = perfilRamp;
        //*********************************************************************

        if(oldInput1!=Input1 || oldInput2!=Input2){
            printInputs(Input1, Input2);

            oldInput1 = Input1;
            oldInput2 = Input2;
        }

        printOutputs(Output1, zcCounter);

        printActualSetpoint((uint16_t)etapa);

        /******* CHEQUEO DE CORRESPONDENCIA PULSOS ENVIADOS/RECIBIDOS ********/
        printPulsesStatus((abs(zcCounter-totalPulsesSent)<=2));
        totalPulsesSent = 0;
        /*********************************************************************/

        // DEBUG
        #ifdef DEBUG
        //Time = (millis()-startTime)/1000.0;
        //Serial.printf("$%.2f %.2f %d %.2f;",Input1, Input2, etapa, perfilRamp);
        #endif

        zcCounter=0;
        nextTime=millis() + WINDOW_1Seg;      
    }
}
