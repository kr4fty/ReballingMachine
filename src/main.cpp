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

#define accel               1 // para acelerar el tiempo, cuanto mas grande mas se acelera, un buen numero es 10

#define WindowSize        200/accel
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

double heatingModeGraph, coolingModeGraph;
double time_heatingMode;
double time_coolingMode;

uint8_t modoFuncionamiento; //1: modo Calefacción, 2: modo en espera, 3: modo Ventilación

double ambTemperature;
 
// the setup function runs once when you press reset or power the board
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    //DEBUG
    #if  defined(DEBUG) || defined(SERIAL_PLOTTER)
    Serial.begin(115200);
    #endif

    // Calentador
    heaterPID.SetSampleTime(WindowSize);
    heaterPID.SetOutputLimits(HEATER_MIN_ANGLE, HEATER_MAX_ANGLE);
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
    printProfileSelection(encoderValue, profileCount);
    windowStartTime = millis();
    // Entro al modo selección. Para seleccionar el actual hago un Click
    uint8_t key=0;
    while(key != SHORT_CLICK){
    // Si hubo pulsación de tecla entonces tomo la posición actual como el seleccionado
        actualTime = millis();
        if (actualTime > windowStartTime){
            readThermocouples(&Input1, &Input2);

            windowStartTime = actualTime + WindowSize; // Leo cada 200mseg.
        }
        // Encoder
        if(myEnc.encoderRotationDetected()){
            // Borro en el display el anterior seleccionado
            clearProfileSelection(oldEncoderValue, profileCount);
            encoderValue = myEnc.readEncoder();
            // Muestro el nuevo seleccionado
            printProfileSelection(encoderValue, profileCount);
            oldEncoderValue = encoderValue;
        }
        key = isButtonClicked();
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

    setPwmPulse(LOWER_HEATER, 0);
    setPwmPulse(EXTRACTOR, 0);
}

// the loop function runs over and over again forever
void loop() {
    actualTime = millis();
    // Detectando el retardo del calentador
    if(myProfile.stageNumber_heatingMode==1){
        if((uint16_t)Input1<=(uint16_t)ambTemperature){
            delayTime = actualTime - startTime;
            //Serial.printf("DELAY: %2.3f\n\n", delayTime);
        }
        else{
            if(millis()<delayTime+250){
                //Serial.printf("\t\tDelay: %d\n", (uint16_t)(delayTime/1000));
            }
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
        
        time_heatingMode =((officialTime)*accel/1000.0);
        time_coolingMode =(officialTime-delayTime)*accel/1000.0;
        if(time_coolingMode<0){
            time_coolingMode = 0;
        }

        // Trazado del perfil ideal
        if((uint16_t)time_heatingMode <= myProfile.time[myProfile.stageNumber_heatingMode] && myProfile.stageNumber_heatingMode < myProfile.length){
           heatingModeGraph = profile_getNextPointOnGraph(time_heatingMode, HEATING_MODE);

        }

        if((uint16_t)time_coolingMode <= myProfile.time[myProfile.stageNumber_coolingMode] && myProfile.stageNumber_coolingMode < myProfile.length){
           coolingModeGraph = profile_getNextPointOnGraph(time_coolingMode, COOLING_MODE);

        }



        //*********************************************************************

        if(isPowerOn){ // En funcionamiento?
        /********************************** PID **********************************/
            // Toma Input se actúa. En la variable Output se guarda lo calculado
            if(time_heatingMode<=myProfile.melting_point_time){
                Setpoint1 = heatingModeGraph;
                heaterPID.Compute();
                setPwmPulse(LOWER_HEATER, Output1);
            }
            else{
                Setpoint1 = 0;
                heaterPID.SetMode(MANUAL);
                setPwmPulse(LOWER_HEATER, 0);
            }
            
            if(time_coolingMode>=myProfile.melting_point_time){
                coolerPID.SetMode(AUTOMATIC);
                Setpoint2 = coolingModeGraph;
                coolerPID.Compute();
                if(Output2 == COOLER_MIN_ANGLE){
                    Output2 = 0;
                }
                setPwmPulse(EXTRACTOR, Output2);
            }
            else{
                Setpoint2 = 0;
                coolerPID.SetMode(MANUAL);
                setPwmPulse(EXTRACTOR, 0);
            }

        /*************************************************************************/
    
        /***************************** CONTROL DE FASE ***************************/
            // Se envía pulso de habilitación del TRIAC
            /*setPulse(Output1);
            if(Output2 == COOLER_MIN_ANGLE){
                Output2 = 0;
            }
            setPwmPulse(Output2);*/
            
            #ifdef ZC_INTERRUPT_FILTER
            // Filtro para detectar falsos Cruces por Cero
            if(millis()>zcNextTime && zcFlag){
                zcFlag = false;
                zcCounter++;
            }
            #endif
        /*************************************************************************/
        }

        #ifdef SERIAL_PLOTTER
        //Serial.printf("$%.2f %d %.2f;",(float)((actualTime-startTime)*accel/1000.0), (uint8_t)myProfile.stageNumber_heatingMode, (float)heatingModeGraph);
        Serial.printf("$%d %d %d %d %d;\n",(uint16_t)Input1, (uint16_t)(Output1*50), (Output2-COOLER_MIN_ANGLE)<0?0:(uint16_t)(Output2-COOLER_MIN_ANGLE), (uint16_t)heatingModeGraph, (uint16_t)coolingModeGraph);
        #endif

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
                //setPulse(LOW);
            }
            else{ // ENCENDIDO
                startZcInterrupt(); // Activo interrupciones de cruce por cero
                if(modoFuncionamiento==HEATING_MODE){
                    heaterPID.SetMode(AUTOMATIC);
                }
                else if(modoFuncionamiento==COOLING_MODE || modoFuncionamiento==STANDBY_MODE){
                    heaterPID.SetMode(AUTOMATIC);
                }
            }
        }
        else if(key == LONG_CLICK){
            esp_restart();
        }
    }
    /*************************************************************************/

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
        printPoint((uint16_t)((actualTime-startTime)*accel/1000), (uint16_t)Input1);
        //printPoint(officialTime/1000, (uint16_t)Input1, ST7735_YELLOW, 1, 4);

        //printOutputs(Output1, zcCounter);
        //printActualSetpoint((uint16_t)stageNumber_heatingMode);
        printZcCount(zcCounter);

        /******* CHEQUEO DE CORRESPONDENCIA PULSOS ENVIADOS/RECIBIDOS ********/
        //printPulsesStatus((abs(zcCounter-totalPulsesSent)<=2));
        totalPulsesSent = 0;
        /*********************************************************************/

        // DEBUG
        #ifdef SERIAL_PLOTTER
        unsigned long Time = (actualTime-startTime)*accel/1000.0;
        //Serial.printf("$%.2f %.2f %d %.2f;",Input1, Input2, stageNumber_heatingMode, heatingModeGraph);
        //Serial.printf("$%.2f %d %.2f;",(float)((actualTime-startTime)*accel/1000.0), (uint8_t)stageNumber_heatingMode, (float)heatingModeGraph);
        //Serial.printf("$%d %d %d %d %d;\n",(uint16_t)Input1, (uint16_t)(Output1*50), (Output2-COOLER_MIN_ANGLE)<0?0:(uint16_t)(Output2-COOLER_MIN_ANGLE), (uint16_t)heatingModeGraph, (uint16_t)coolingModeGraph);
        #endif

        zcCounter=0;
        nextTime=millis() + WINDOW_1Seg/accel;
    }
}
