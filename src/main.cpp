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
#include "tclock.h"

#define WindowSize        200
#define WindowSize_cooler 200
#define DELAY_TIME         10  // Retraso de la curva real a la del perfil
unsigned long windowStartTime;
unsigned long       startTime;
unsigned long        nextTime;
unsigned long      actualTime;
unsigned long       delayTime;
unsigned long    maxTemp_time; // tiempo en el que la Temperatura es Máxima
unsigned long    officialTime; // Tiempo lineal de funcionamiento, sin interrupciones

bool isPowerOn;
unsigned long encoderValue;
unsigned long oldEncoderValue;

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
PID heaterPID = PID(&Input1, &Output1, &Setpoint1, KP_HEATER, KI_HEATER, KD_HEATER, DIRECT);
PID coolerPID = PID(&Input2, &Output2, &Setpoint2, KP_COOLER, KI_COOLER, KD_COOLER, REVERSE);

float perfilRamp;
uint16_t profileTime;
uint8_t etapa=1;
uint8_t modoFuncionamiento; //1: modo Calefacción, 2: modo en espera, 3: modo Ventilación

double ambTemperature;
 
// the setup function runs once when you press reset or power the board
void setup() {
    //DEBUG
    #if  defined(DEBUG) || defined(SERIAL_PLOTER)
    Serial.begin(115200);
    #endif

    // Calentador
    heaterPID.SetSampleTime(WindowSize);
    heaterPID.SetOutputLimits(0, 1);
    heaterPID.SetMode(AUTOMATIC);
    // Extractor
    coolerPID.SetSampleTime(WindowSize_cooler);
    coolerPID.SetOutputLimits(COOLER_MIN_ANGLE, COOLER_MAX_ANGLE); // para el angulo de disparo es 180-angulo
    coolerPID.SetMode(AUTOMATIC);

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
    // Busco los Perfiles disponibles y los inicializo
    profile_initializeProfiles();
    // Imprimo menu mostrando los posible Perfiles a seleccionar
    printProfiles(profileNames, profileCount);
    // Configuro el encoder de acuerdo a los Perfiles disponibles
    encoder_setBasicParameters(0,profileCount-1, true, -1, 10);
    encoderValue = oldEncoderValue = myEnc.readEncoder();
    // Marco el primer elemento como seleccionado
    printProfileSelection(encoderValue);
    windowStartTime = millis();
    // Entro al modo selección. Para seleccionar el actual hago un Click
    while(!isButtonClicked()){
    // Si hubo pulsación de tecla entonces tomo la posición actual como el seleccionado
        actualTime = millis();
        if (actualTime > windowStartTime){
            readThermocouples(&Input1, &Input2);

            windowStartTime = actualTime + WindowSize; // Leo cada 200mseg.
        }
        // Encoder
        if(myEnc.encoderRotationDetected()){
            // Borro en el display el anterior seleccionado
            clearProfileSelection(oldEncoderValue);
            encoderValue = myEnc.readEncoder();
            // Muestro el nuevo seleccionado
            printProfileSelection(encoderValue);
            oldEncoderValue = encoderValue;
        }
    }
    profile_selectProfile(encoderValue);
    //printSelectedProfile(myProfile.name, myProfile.time, myProfile.temperature, myProfile.length);
    /*************************************************************************/

    //printLavels();
    printFrameBase(myProfile.time, myProfile.temperature, myProfile.length);

    // Inicializo los Timers que realizaran el pulso que activa el Triac 
    initPwmPulseSettings();

    // Realizo cálculos iniciales de acuerdo al Perfil seleccionado
    actualTime = millis();
    nextTime=actualTime + WINDOW_1Seg;
    startTime = actualTime;
    profile_preCalculate(Input1, (uint16_t)((actualTime-startTime)/1000));

    // Entramos en modo Calentamiento
    modoFuncionamiento = HEATING_MODE;

    // Guardo la temperatura en el ambiente
    ambTemperature = Input1;
    
    #ifdef DEBUG
    Serial.printf("Ingresando al Modo Calentamiento\n");
    Serial.printf("Arranque: %d\n",(uint16_t)((millis()-startTime)/1000));
    #endif

    windowStartTime = actualTime;
    isPowerOn = true;
    printSystemStatus(isPowerOn);
    delayTime = 0;
    printProfileName(myProfile.name, ST7735_YELLOW);

    setPwmPulse(0);
    setPulse(LOW);
}

// the loop function runs over and over again forever
void loop() {
    actualTime = millis();
    // Detectando el retardo del calentador
    if(etapa==1){
        if((uint16_t)Input1<=(uint16_t)ambTemperature){
            delayTime = actualTime - startTime;
            //Serial.printf("DELAY: %2.3f\n\n", delayTime);
        }
    }
    // Senso la temperatura cada 200mSeg 
    // (frecuencia máxima a la que lee el sensor max6675)
    if (actualTime > windowStartTime){
        //readThermocouples(&Input1, &Input2);
        readThermocouple(&Input1);
        Input2 = Input1;
        
        // Perfil térmico *****************************************************
        officialTime = actualTime - startTime;

        profileTime =(uint16_t)((actualTime-startTime)/1000);
        
        /*if(modoFuncionamiento == HEATING_MODE){// || modoFuncionamiento == STANDBY_MODE){
            profileTime =(uint16_t)((actualTime-startTime)/1000);
        }
        else if(modoFuncionamiento == COOLING_MODE){
            profileTime =(uint16_t)((actualTime-startTime-delayTime)/1000);
        }*/

        // Trazado del perfil ideal
        if(profileTime < myProfile.time[etapa] && etapa < myProfile.length){
            perfilRamp = tempSlope[etapa-1]*(profileTime-myProfile.time[etapa-1]) + myProfile.temperature[etapa-1];
            //Serial.printf("%d - %d\n", (profileTime+1), myProfile.time[etapa]);
            if((uint16_t)(profileTime+1)==myProfile.time[etapa]){
                // Paso a la siguiente etapa
                etapa++;
            }

        }
        else{
            perfilRamp = ambTemperature;
        }

        if(modoFuncionamiento==HEATING_MODE){
            Setpoint1 = perfilRamp;
        }
        else if(modoFuncionamiento==STANDBY_MODE){
            Setpoint1 = 0;
            Setpoint2 = perfilRamp;
        }
        else if(modoFuncionamiento==COOLING_MODE){
            Setpoint2 = perfilRamp;
        }
        //*********************************************************************

        // modos de trabajo
        switch (modoFuncionamiento)
        {
            case HEATING_MODE:                
                if((uint16_t)perfilRamp <= myProfile.melting_point){
                    maxTemp_time = officialTime;
                    //Serial.printf("Modo %s Tiempo: %d  Etapa: %d Ramp: %.2f max_Time %d\n", "CALEFACCIÓN", (uint16_t)officialTime/1000, etapa, perfilRamp, maxTemp_time);   
                }
                else{
                    modoFuncionamiento = STANDBY_MODE; // cambio a Modo en espera
                    //Serial.print("Cambiando a modo en Espera\n");
                }
                break;
            case STANDBY_MODE:
                if(officialTime >= (maxTemp_time+delayTime)){
                    modoFuncionamiento = COOLING_MODE; // Cambio a Modo Enfriamiento activo
                    //Serial.print("Cambiando a modo Cooling\n");
                }
                else{
                    //Serial.println("En espera");
                }
                break;
            case COOLING_MODE:
                if((uint16_t)perfilRamp <= myProfile.temperature[myProfile.length-1]){
                    modoFuncionamiento = STOP_MODE;
                    //Serial.print("Deteniendo... \n");
                }
                else{
                    //Serial.printf("Modo %s Tiempo: %d  Etapa: %d\n", "ENFRIAMIENTO", (uint16_t)officialTime/1000, etapa);
                }
                break;
            default:
                break;
        }

        windowStartTime = actualTime + WindowSize;
    }

    /****************************** START/STOP *******************************/
    uint8_t key=isButtonClicked();
    if(key) {
        if(key == SHORT_CLICK){
            isPowerOn = not isPowerOn;

            printSystemStatus(isPowerOn);

            if(!isPowerOn) { // APAGADO
                heaterPID.SetMode(MANUAL);
                stopZcInterrupt(); // Desactivo interrupciones de cruce por cero
                coolerPID.SetMode(MANUAL);
                setPulse(LOW);
            }
            else{ // ENCENDIDO
                startZcInterrupt(); // Activo interrupciones de cruce por cero
                if(modoFuncionamiento==HEATING_MODE){
                    heaterPID.SetMode(AUTOMATIC);
                }
                else if(modoFuncionamiento==COOLING_MODE){
                    heaterPID.SetMode(AUTOMATIC);
                }
            }
        }
        else if(key == LONG_CLICK){
            esp_restart();
        }
    }
    /*************************************************************************/

    if(isPowerOn){ // En funcionamiento?
    /********************************** PID **********************************/
        // Toma Input y actúa. En la variable Output se guarda lo calculado
        if(modoFuncionamiento==HEATING_MODE){
            heaterPID.Compute();
        }
        else if(modoFuncionamiento==STANDBY_MODE){
            heaterPID.SetMode(MANUAL);
            coolerPID.Compute();
        }
        else if(modoFuncionamiento==COOLING_MODE) {
            coolerPID.Compute();
        }
    /*************************************************************************/

    /***************************** CONTROL DE FASE ***************************/
        // Se envía pulso de habilitación del TRIAC
        if(modoFuncionamiento==HEATING_MODE){
            setPulse(Output1);
            setPwmPulse(0);
        }
        else if(modoFuncionamiento==STANDBY_MODE){
            setPulse(LOW);
            setPwmPulse(Output2);
        }
        else if(modoFuncionamiento==COOLING_MODE){
            setPulse(LOW);
            if(Output2==COOLER_MIN_ANGLE){
                Output2 = 60;
            }
            setPwmPulse(Output2);
        }
        #ifdef ZC_INTERRUPT_FILTER
        // Filtro para detectar falsos Cruces por Cero
        if(millis()>zcNextTime && zcFlag){
            zcFlag = false;
            zcCounter++;
        }
        #endif
    /*************************************************************************/
    }

    actualTime = millis();
    if(actualTime>nextTime){  // Una vez por segundo
        // Actualizo el reloj
        clock_update();
        // imprimo en el lcd
        printTime(clock_get_hours(), clock_get_minutes(), clock_get_seconds());

        if(oldInput1!=Input1 || oldInput2!=Input2){
            //printInputs(Input1, Input2);
            printTemperatures(Input1, Input2);

            oldInput1 = Input1;
            oldInput2 = Input2;
        }

        // Voy gráficando la evolución de la temperatura con el tiempo
        printPoint((uint16_t)(officialTime/1000), (uint16_t)Input1);
        //printPoint(officialTime/1000, (uint16_t)Input1, ST7735_YELLOW, 1, 4);

        //printOutputs(Output1, zcCounter);
        //printActualSetpoint((uint16_t)etapa);
        printZcCount(zcCounter);

        /******* CHEQUEO DE CORRESPONDENCIA PULSOS ENVIADOS/RECIBIDOS ********/
        //printPulsesStatus((abs(zcCounter-totalPulsesSent)<=2));
        totalPulsesSent = 0;
        /*********************************************************************/

        // DEBUG
        #ifdef SERIAL_PLOTER
        unsigned long Time = (actualTime-startTime)/1000.0;
        //Serial.printf("$%.2f %.2f %d %.2f;",Input1, Input2, etapa, perfilRamp);
        Serial.printf("$%d %d %d %d;\n",(uint16_t)Input1, (uint16_t)(Output1*50), (Output2-COOLER_MIN_ANGLE)<0?0:(uint16_t)(Output2-COOLER_MIN_ANGLE), (uint16_t)perfilRamp);
        #endif

        zcCounter=0;
        nextTime=millis() + WINDOW_1Seg;
    }
}
