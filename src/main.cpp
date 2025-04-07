/******************************************************************************
*
*     Reballing Machine
*
*     Autor: Tapia Velasquez Favio
*     Version: 0.09.3
*                   Posibilidad de seleccionar entre varios Perfiles guardados
*                   Se dibuja el perfil seleccionado
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
    // Imprimo menu mostrando los posible Perfiles a seleccionar
    printProfiles(profileNames, profileCount);
    // Configuro el encoder de acuerdo a los Perfiles disponibles
    encoder_setBasicParameters(0,profileCount-1, true, 0, 10);
    encoderValue = oldEncoderValue = myEnc.readEncoder();
    // Marco el primer elemento como seleccionado
    printProfileSelection(encoderValue+1);
    windowStartTime = millis();
    // Entro al modo selección. Para seleccionar el actual hago un Click
    do{ 
        if (millis() > windowStartTime){
            readThermocouples(&Input1, &Input2);

            windowStartTime = millis() + WindowSize; // Leo cada 200mseg.
        }
        // Encoder
        if(myEnc.encoderRotationDetected()){
            // borro el anterior
            clearProfileSelection(oldEncoderValue+1);
            encoderValue = myEnc.readEncoder();
            printProfileSelection(encoderValue+1);
            oldEncoderValue = encoderValue;
        }
    }while(!isButtonClicked());// Si hubo pulsación de tecla entonces tomo la posición actual como el seleccionado
    profile_selectProfile(encoderValue);
    //printSelectedProfile(myProfile.name, myProfile.time, myProfile.temperature, myProfile.length);
    /*************************************************************************/

    //printLavels();
    printFrameBase(myProfile.time, myProfile.temperature, myProfile.length);

    // Inicializo los Timers que realizaran el pulso que activa el Triac 
    initPwmPulseSettings();

    // Realizo cálculos iniciales de acuerdo al Perfil seleccionado
    nextTime=millis() + WINDOW_1Seg;
    startTime = millis();
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

            // Voy gráficando la evolución de la temperatura con el tiempo
            printPoint(tiempo, (uint16_t)Input1);
        }
        else{
            perfilRamp = 0;
        }
        Setpoint1 = perfilRamp;
        //*********************************************************************

        if(oldInput1!=Input1 || oldInput2!=Input2){
            //printInputs(Input1, Input2);
            printTemperatures(Input1, Input2);

            oldInput1 = Input1;
            oldInput2 = Input2;
        }

        //printOutputs(Output1, zcCounter);
        //printActualSetpoint((uint16_t)etapa);
        printZcCount(zcCounter);

        /******* CHEQUEO DE CORRESPONDENCIA PULSOS ENVIADOS/RECIBIDOS ********/
        //printPulsesStatus((abs(zcCounter-totalPulsesSent)<=2));
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
