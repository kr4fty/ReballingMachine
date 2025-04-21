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

// Función en ensamblador para poner en alto un pin específico
void writePinHigh(uint32_t pin) {
    uint32_t mask = 1 << pin;
    uint32_t *gpio_out = (uint32_t *)0x3FF44004; // Dirección del registro de salida GPIO
    *gpio_out |= mask;
}

// Función en ensamblador para poner en bajo un pin específico
void writePinLow(uint32_t pin) {
    uint32_t mask = 1 << pin;
    uint32_t *gpio_out = (uint32_t *)0x3FF44004; // Dirección del registro de salida GPIO
    *gpio_out &= ~mask;

    //POR AHORA LO VEO MAS FACTIBLE QUE ESTE AQUI, lo malo es que si estan habilidas las tres salidas, se le sumaran 3 veces los los pulsos
    totalPulsesSent += 1;
}

void IRAM_ATTR zc_ISR() {
    // Reiniciar el contador de interrupciones del Timer
    currentInterruptCount = 0;

    // Forzamos a reiniciar los estados de las salidas
    writePinLow(SALIDA_1);
    writePinLow(SALIDA_3);
    writePinLow(SALIDA_2);

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
    if(isItEnabled[LOWER_HEATER]){
        if(currentInterruptCount == outputAngles[LOWER_HEATER]){
            writePinHigh(SALIDA_1);
        }
        if(currentInterruptCount == (outputAngles[LOWER_HEATER]+1)){
            writePinLow(SALIDA_1);
        }
    }
    if(isItEnabled[UPPER_HEATER]){
        if(currentInterruptCount == outputAngles[UPPER_HEATER]){
            writePinHigh(SALIDA_3);
        }
        if(currentInterruptCount == (outputAngles[UPPER_HEATER]+1)){
            writePinLow(SALIDA_3);
        }
    }
    if(isItEnabled[EXTRACTOR]){
        if(currentInterruptCount == outputAngles[EXTRACTOR]){
            writePinHigh(SALIDA_2);
        }
        if(currentInterruptCount == (outputAngles[EXTRACTOR]+1)){
            writePinLow(SALIDA_2);
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
            /*  El area en el seno no varia linealmente con el angulo, este se 
                parece mas a una tangente hiperbólica en lugar de ser una recta
                con pendiente positiva.

                Para calcular el area desde 0 a X1 del seno es: Area(x1)=1−cos(x1)
                Luego x=arccos(1−p), con p= porcentaje que deseamos

                Area_% = Area(x1)/Area(180) = (1 - cos(x1)) / (1 - cos(180)) = (1 - cos(x1)) / 2

                angulo = acos(1 - Area_%) * (180.0 / PI), nos dará el angulo real para obtener la proporción de area deseada
            */
            double angle = acos(1-output/90.0)*(180.0/M_PI);

            uint16_t pulseTime = map(angle, 0, 180, 180, 10); // Lo pongo dentro de los limites 
            outputAngles[outputIndex] = pulseTime*ESCALA; // Almacenar el tiempo para la salida correspondiente

            isItEnabled[outputIndex] = true; // Marcar la salida como habilitada
        }
        else { //output == 0
            // Se vio en el osc que ahi esta aproximadamente el centro del pulso de ZC
            //outputAngles[outputIndex] = 172; // CERO
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
