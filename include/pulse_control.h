/******************************************************************************
 * 
 *  pulse_control.h
 * 
 *  Encargado de configurar los timers (Timer1 y 2),  para enviar un pulso por 
 *  cada ciclo loop, siempre y cuando llegue un pulso por el pin de Detección
 *  de Cruce por Cero (ZC_PIN). Dicho pulso sera enviado a un Angulo de Disparo
 *  determinado en la variable Output. La misma es entregada por la función de 
 *  calculo del PID.
 * 
 *  La duración del pulso es de unos 15uSeg (G_PULSE_WIDTH). Tiempo suficiente 
 *  para activar el tiristor
 * 
 *  Utilizando API v3.x del Timer
 * 
 *  Autor: Tapia Velasquez Favio
 * 
 *****************************************************************************/

#ifndef PULSE_CONTROL_H
#define PULSE_CONTROL_H

#include "config.h"

#define   TIMER1 1
#define   TIMER2 2
hw_timer_t *Timer1 = NULL;
hw_timer_t *Timer2 = NULL;
uint16_t  timerTicks;

uint64_t  pulseDelay=0;
double    pulseDelayTransformed;
volatile uint16_t  zcCounter=0;
volatile bool isUp;
volatile uint16_t totalPulsesSent=0;

#ifdef ZC_INTERRUPT_FILTER
  uint32_t zcNextTime=0;
  bool zcFlag=false;
#endif

void IRAM_ATTR zc_ISR()
{
    #ifdef ZC_INTERRUPT_FILTER
    // Filtro para falsos positivos
    if(!zcFlag){
        zcFlag = true;
        zcNextTime=millis()+WINDOW_INTERRUPT;
        timerRestart(Timer1);
        timerRestart(Timer2);
        timerAlarm(Timer1, timerTicks, false, 0);
        timerAlarm(Timer2, timerTicks + G_PULSE_WIDTH, false, 0);
    }
    #else
        zcCounter += 1;
        timerRestart(Timer1);
        timerRestart(Timer2);
        timerAlarm(Timer1, timerTicks, false, 0);
        timerAlarm(Timer2, timerTicks + G_PULSE_WIDTH, false, 0);
    #endif
}

void IRAM_ATTR Timer1_ISR()
{
    if(timerTicks<(t10mSEG-G_PULSE_WIDTH)){ // Si es 0 lo mantengo apagado siempre
        digitalWrite(RELAY_PIN, HIGH);      // => nunca entraría aquí en ese caso
        if(digitalRead(RELAY_PIN)){
            isUp = true;
        }
    }
}

void IRAM_ATTR Timer2_ISR()
{
    if(timerTicks>(ZC_PULSE_WIDTH/2)){ // Si es 180 lo mantengo encendido siempre
        digitalWrite(RELAY_PIN, LOW);  // => nunca entraría aquí en ese caso
        if(isUp && !digitalRead(RELAY_PIN)){
            isUp = false;
            totalPulsesSent += 1;
        }
    }
}

void initPwmPulseSettings()
{
    attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_ISR,RISING);

    Timer1 = timerBegin(1000000); // APIv3 1Mhz
    timerAttachInterrupt(Timer1, &Timer1_ISR); // APIv3
    
    Timer2 = timerBegin(1000000); // APIv3 1Mhz
    timerAttachInterrupt(Timer2, &Timer2_ISR); // APIv3
}

void setPwmPulse(double output)
{
    // Para obtener la proporción correcta de la superficie del semiciclo
    // La superficie del SENO no cambia linealmente con el angulo
    pulseDelayTransformed = acos(1-output/90.0)*(180.0/M_PI);
                                                           
    pulseDelay = t10mSEG-(uint64_t)map(pulseDelayTransformed, 0, ZC_MAX_ANGLE, G_PULSE_WIDTH, t10mSEG-ZC_PULSE_WIDTH/2);
    // Calculo de los timerTick para el pulseDelay dado
    // timerTicks = (pulseDelay * 1x10⁻⁶ * 80x10⁶) / PRESCALER = pulseDelay
    timerTicks = (uint16_t)pulseDelay;
}

void stopZcInterrupt()
{
    detachInterrupt(digitalPinToInterrupt(ZC_PIN));
    timerDetachInterrupt(Timer1);
    timerDetachInterrupt(Timer2);
    digitalWrite(RELAY_PIN, LOW);
}

void startZcInterrupt()
{
    initPwmPulseSettings();
}
#endif
