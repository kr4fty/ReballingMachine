#include <Arduino.h>
#include "config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <AiEsp32RotaryEncoder.h>
//#include <math.h>

//#define _USE_MATH_DEFINES

AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(ENCODER_PIN_A, ENCODER_PIN_B, BUTTON, ENCODER_VCC_PIN, ENCODER_STEPS);
void IRAM_ATTR readEncoderISR(){
	encoder.readEncoder_ISR();
}

#define TIMER1 1
#define TIMER2 2
hw_timer_t *Timer1 = NULL;
hw_timer_t *Timer2 = NULL;
uint16_t timerTicks; 

void IRAM_ATTR Timer1_ISR()
{
    if(timerTicks<(t10mSEG-G_PULSE_WIDTH)) 
      digitalWrite(RELAY_PIN, HIGH);    
}
void IRAM_ATTR Timer2_ISR()
{
  if(timerTicks>(ZC_PULSE_WIDTH/2))
    digitalWrite(RELAY_PIN, LOW);
}


long int encoderCounter=90, oldEncoderCounter=-1;

#ifdef ZC_INTERRUPT_FILTER
uint32_t zcNextTime=0;
bool zcFlag=false;
#endif
int32_t nextTime; 
uint64_t pulseDelay=0;
double turnedPulseDelay;
long int zcCounter=0;

Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void IRAM_ATTR zc_isr() {
  // Filtro para falsos positivos
  #ifdef ZC_INTERRUPT_FILTER
  if(!zcFlag){
    zcFlag = true;
    zcNextTime=millis()+WINDOW_INTERRUPT;
    timerRestart(Timer1);
    timerRestart(Timer2);
    timerAlarmEnable(Timer1);
    timerAlarmEnable(Timer2);
  }
  #else
    zcCounter++;
    timerRestart(Timer1);
    timerRestart(Timer2);
    timerAlarmEnable(Timer1);
    timerAlarmEnable(Timer2);
  #endif
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(ZC_PIN, INPUT);

  // Use this initializer if using a 1.8" TFT screen:
  lcd.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Para sobre escribir puntos
  lcd.fillScreen(ST77XX_BLACK);
  lcd.setRotation(1);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.setCursor(6*0*MIDLE_TEXT, 1*8*MIDLE_TEXT);
  lcd.print("Cruces: ");
  lcd.setCursor(6*0*MIDLE_TEXT, 2.5*8*MIDLE_TEXT);
  lcd.print("Dutty: ");
  lcd.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  
  encoder.begin();
  encoder.setup(readEncoderISR);
  bool circleValues =false;
  encoder.setBoundaries(0, ZC_MAX_ANGLE, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  //encoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
	encoder.setAcceleration(100); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  encoder.setEncoderValue(90);

  attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_isr,RISING);

  nextTime=millis() + WINDOW_1Seg;

  Timer1 = timerBegin(TIMER1, PRESCALER, true);
  timerAttachInterrupt(Timer1, &Timer1_ISR, true);
  timerAlarmWrite(Timer1, 1000000, false);
  //timerAlarmEnable(Timer1);

  Timer2 = timerBegin(TIMER2, PRESCALER, true);
  timerAttachInterrupt(Timer2, &Timer2_ISR, true);
  timerAlarmWrite(Timer2, 1000000, false);
  //timerAlarmEnable(Timer2);

}

void loop(){

  if (encoder.encoderChanged()){ // Para el control de fase, seteo manual
    encoderCounter = encoder.readEncoder();
  }

  if (oldEncoderCounter != encoderCounter) {
    lcd.setCursor(1*6*GIGANT_TEXT, 1.5*7*GIGANT_TEXT);
    lcd.setTextSize(GIGANT_TEXT);
    lcd.printf("%3d", encoderCounter);
    oldEncoderCounter = encoderCounter;
    
    turnedPulseDelay = acos(1-encoderCounter/90.0)*(180.0/M_PI); // Para obtener la proporción correcta de la superficie del semiciclo
    lcd.setTextSize(MIDLE_TEXT);                                 // La superficie del SENO no cambia linealmente con el angulo
    lcd.setCursor(8*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.printf("%3.1f", turnedPulseDelay);

    pulseDelay = t10mSEG-map(turnedPulseDelay, 0, ZC_MAX_ANGLE, G_PULSE_WIDTH, t10mSEG-ZC_PULSE_WIDTH/2);
  //output=180, siempre encendido => t10mSEG-(t10mSEG-ZC_PULSE_WIDTH/2) = ZC_PULSE_WIDTH/2
  //output=0,   siempre apagado   => t10mSEG-G_PULSE_WIDTH
    lcd.setTextSize(MIDLE_TEXT); 
    lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.printf("%4d", pulseDelay);


    // Calculo de los timerTick para el pulseDelay dado
    // timerTicks = (pulseDelay * 1x10⁻⁶ * 80x10⁶) / PRESCALER = pulseDelay
    timerTicks = (uint16_t)pulseDelay;
    timerAlarmWrite(Timer1, timerTicks, false);
    timerAlarmWrite(Timer2, timerTicks+G_PULSE_WIDTH, false);
    /* Fin Calculo timerTicks*/
    //lcd.setCursor(6*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    //lcd.printf("%d",timerTicks);
  }

  /***************************** CONTROL DE FASE ****************************/
  //pulseDelay = t10mSEG-(uint64_t)map(encoderCounter, 0, ZC_MAX_ANGLE, ZC_PULSE_WIDTH/2, t10mSEG-ZC_PULSE_WIDTH/2-G_PULSE_WIDTH);
  // Filtro para falsos positivos
  #ifdef ZC_INTERRUPT_FILTER
  if(millis()>zcNextTime && zcFlag){
    zcFlag = false;
    zcCounter++;
  }
  #endif
  /*************************************************************************/

  // cuento interrupsiones de cruce por cero, por segundo
  if(millis()>nextTime){
    lcd.setCursor(8*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.printf("%3d", zcCounter);
    zcCounter=0;
    nextTime=millis() + WINDOW_1Seg;
  }
}
 