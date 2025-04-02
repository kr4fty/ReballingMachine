/* Generando un PWM mediante el uso de los Timers del ESP32 (son 4)
 *
 * No se utiliza AnalogWrite debido a que yo necesito que se active a un tiempo
 * despues de llegado una interrupcion y no en 0.
 * La duracion del pulso, tambien es muy pequeña
 *
 * Utilizo una interrupcion del timer, Alarma, para dejar pasar un tiempo antes
 * de poner un HIGH y luego utlizo un segundo Timer para llebar el pulso a LOW
 *
 * Cada alarma de los timer se activan una vez, por lo que si no llega la
 * interrupcion exterior, jamas se envia el pulso.
 *
 * AUTOR: Tapia Velasquez Favio
 */


#include<Arduino.h>

#define LED LED_BUILTIN
hw_timer_t *Timer0_Cfg = NULL;
hw_timer_t *Timer1_Cfg = NULL;
hw_timer_t *Timer2_Cfg = NULL;
 
void IRAM_ATTR Timer0_ISR()
{
    timerRestart(Timer1_Cfg);
    timerRestart(Timer2_Cfg);
    timerAlarmEnable(Timer1_Cfg);
    timerAlarmEnable(Timer2_Cfg);
}
void IRAM_ATTR Timer1_ISR()
{
    digitalWrite(LED, HIGH);
}
void IRAM_ATTR Timer2_ISR()
{
    digitalWrite(LED, LOW);
}
void setup()
{
    pinMode(LED, OUTPUT);

    Timer0_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000000, true);
    timerAlarmEnable(Timer0_Cfg);

    Timer1_Cfg = timerBegin(1, 80, true);
    timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
    timerAlarmWrite(Timer1_Cfg, 250000, false);
    //timerAlarmEnable(Timer1_Cfg);

    Timer2_Cfg = timerBegin(2, 80, true);
    timerAttachInterrupt(Timer2_Cfg, &Timer2_ISR, true);
    timerAlarmWrite(Timer2_Cfg, 750000, false);
    //timerAlarmEnable(Timer2_Cfg);
}
void loop()
{
    // Do Nothing!
}
