/******************************************************************************
 * 
 *  pulse_control.h
 * 
 *  Encargado de configurar el Timer para enviar un pulso por, siempre y cuando
 *  llegue un pulso por el pin de Detección de Cruce por Cero (ZC_PIN).
 * 
 *  Dicho pulso será enviado con un Angulo de Disparo determinado, que sera 
 *  provista en la variable Output. Esta variable es calculada por la función
 *  de calculo del PID.
 * 
 *  La duración del pulso es de unos 55.55 uSeg. Tiempo suficiente para activar
 *  el Triac.
 * 
 *  Utilizando API v3.x del Timer
 * 
 *  Autor: Tapia Velasquez Favio
 * 
 *****************************************************************************/

#ifndef PULSE_CONTROL_H
#define PULSE_CONTROL_H
 
#include "config.h"

#define MAX_OUTPUTS 3 // Número máximo de salidas

#define TIMER_FREQUENCY 1800000 // 1 MHz
#define INTERRUPT_FREQUENCY 100 // 100 Hz
#define MAXIMUN_ANGLE       180 // 180º
#define ESCALA                1 // Mas grande para mayor resolución, por lo que el ancho del PULSO G sera mas delgado, pero mas uso del mcu
#define TIMER_INTERVAL (uint16_t)(TIMER_FREQUENCY/(INTERRUPT_FREQUENCY*ESCALA*MAXIMUN_ANGLE))

hw_timer_t *Timer = NULL;
volatile uint16_t outputAngles[MAX_OUTPUTS]; // Ángulos de disparo para cada salida
volatile uint16_t currentInterruptCount = 0; // Contador de interrupciones
volatile bool isItEnabled[MAX_OUTPUTS] = {false, false, false}; // Estan habilidos

volatile uint16_t  zcCounter=0;
volatile bool isUp;
volatile uint16_t totalPulsesSent=0;

void IRAM_ATTR zc_ISR() {
    // Reiniciar el contador de interrupciones del Timer
    currentInterruptCount = 0;

    // Reiniciar los estados de las salidas
    for (int i = 0; i < MAX_OUTPUTS; i++) {
        switch (i)
        {
            case LOWER_HEATER:
                digitalWrite(SALIDA_1, LOW); // Activar la salida 1
                break;
            case UPPER_HEATER:
                digitalWrite(SALIDA_3, LOW); // Activar la salida 2
                break;
            case EXTRACTOR:
                digitalWrite(SALIDA_2, LOW); // Activar la salida 3
                break;
        
            default:
                break;
        }
    }

    #ifdef ZC_INTERRUPT_FILTER    
    // Filtro para falsos positivos
    if(!zcFlag){
        zcFlag = true;
        zcNextTime=millis()+WINDOW_INTERRUPT;
    }
    #else
        zcCounter += 1;
        timerRestart(Timer);
    #endif
}

void IRAM_ATTR Timer_ISR() {
    currentInterruptCount += 1;

    // Verificar si se debe activar alguna salida
    for (int i = 0; i < MAX_OUTPUTS; i++) {
        if(isItEnabled[i]){ // Esta habilitado la salida actual?
            if (currentInterruptCount == outputAngles[i]) {
                switch (i)
                {
                    case LOWER_HEATER:
                        digitalWrite(SALIDA_1, HIGH); // Activar la salida 1
                        break;
                    case UPPER_HEATER:
                        digitalWrite(SALIDA_3, HIGH); // Activar la salida 2
                        break;
                    case EXTRACTOR:
                        digitalWrite(SALIDA_2, HIGH); // Activar la salida 3
                        break;
                
                    default:
                        break;
                }
            }
            // Desactivar la salida después de G_PULSE_WIDTH
            if (currentInterruptCount == outputAngles[i] + 1) { //<-- +1: sera el ancho G_PULSE_WIDTH 
                switch (i)
                {
                    case LOWER_HEATER:
                        digitalWrite(SALIDA_1, LOW); // Desactivar la salida 1
                        break;
                    case UPPER_HEATER:
                        digitalWrite(SALIDA_3, LOW); // Desactivar la salida 2
                        break;
                    case EXTRACTOR:
                        digitalWrite(SALIDA_2, LOW); // Desactivar la salida 3
                        break;
                
                    default:
                        break;
                }
                totalPulsesSent += 1;
            }
        }
    }
}

void initPwmPulseSettings() {
    pinMode(ZC_PIN, INPUT);
    
    pinMode(SALIDA_1, OUTPUT);
    pinMode(SALIDA_2, OUTPUT);
    pinMode(SALIDA_3, OUTPUT);

    digitalWrite(SALIDA_1, LOW);
    digitalWrite(SALIDA_2, LOW);
    digitalWrite(SALIDA_3, LOW);

    attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_ISR, RISING);

    // Inicializar el temporizador con la nueva API
    Timer = timerBegin(TIMER_FREQUENCY); // Configura el temporizador a 1.8 MHz
    timerAttachInterrupt(Timer, &Timer_ISR); // Adjuntar la interrupción
    timerAlarm(Timer, TIMER_INTERVAL, true, 0); // Configurar el temporizador para que interrumpa cada TIMER_INTERVAL
}

void setPwmPulse(uint8_t outputIndex, double output) {
    if (outputIndex < MAX_OUTPUTS) {
        // Calcular el tiempo de activación basado en el ángulo de disparo
        if(output){ // distinto de 0
            uint16_t pulseTime = map(output, 0, 180, 180, 10); // Lo pongo dentro de los limites 
            outputAngles[outputIndex] = pulseTime*ESCALA; // Almacenar el tiempo para la salida correspondiente

            isItEnabled[outputIndex] = true; // Marcar la salida como habilitada
        }
        else { //output == 0
            // Se vio en el osc que ahi esta aproximadamente el centro del pulso de ZC
            outputAngles[outputIndex] = 172; // CERO
            isItEnabled[outputIndex] = false; // Marco como deshabilitada para ocupar menos mcu en la ISR del timer
        }


    }
}

void disablePwm(uint8_t outputIndex)
{
    isItEnabled[outputIndex] = false;
    switch (outputIndex)
    {
        case LOWER_HEATER:
            digitalWrite(SALIDA_1, LOW); // Desactivar la salida 1
            break;
        case UPPER_HEATER:
            digitalWrite(SALIDA_3, LOW); // Desactivar la salida 2
            break;
        case EXTRACTOR:
            digitalWrite(SALIDA_2, LOW); // Desactivar la salida 3
            break;
    
        default:
            break;
    }
}

void stopZcInterrupt()
{
    detachInterrupt(digitalPinToInterrupt(ZC_PIN));
    timerDetachInterrupt(Timer);
    isItEnabled[LOWER_HEATER] = false;
    isItEnabled[UPPER_HEATER] = false;
    isItEnabled[EXTRACTOR] = false;
    digitalWrite(SALIDA_1, LOW);
    digitalWrite(SALIDA_2, LOW);
    digitalWrite(SALIDA_3, LOW);

}

void startZcInterrupt()
{
    initPwmPulseSettings();
}
#endif
