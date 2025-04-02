/* Aplicacion Dimmer hecho con un ESP32, haciendo uso de un hardware detector
 * de cruce por cero y otro para la etapa de potencia
 *
 * AUTOR: Tapia Velasquez Favio
 */


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

long int encoderCounter=90, oldEncoderCounter=-1;

#ifdef ZC_INTERRUPT_FILTER
  uint32_t zcNextTime=0;
#endif
int32_t nextTime; 
uint64_t pulseOn, pulseOff, pulseDelay=0;
double turnedPulseDelay;
bool zcFlag=false;
long int zcCounter=0;

Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void IRAM_ATTR zc_isr() {
  // Filtro para falsos positivos
  #ifdef ZC_INTERRUPT_FILTER
  if(!zcFlag){
    zcFlag = true;
    zcNextTime=millis()+WINDOW_INTERRUPT;
    pulseOn = micros()+pulseDelay;
    pulseOff = pulseOn+G_PULSE_WIDTH
  }
  #else
    zcCounter++;
    pulseOn = micros()+pulseDelay;
    pulseOff = pulseOn+G_PULSE_WIDTH;
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

    pulseDelay = t10mSEG-(uint64_t)map(turnedPulseDelay, 0, ZC_MAX_ANGLE, G_PULSE_WIDTH, t10mSEG-ZC_PULSE_WIDTH/2);
  //output=180, siempre encendido => t10mSEG-(t10mSEG-ZC_PULSE_WIDTH/2) = ZC_PULSE_WIDTH/2
  //output=0,   siempre apagado   => t10mSEG-G_PULSE_WIDTH
    lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.printf("%5d", pulseDelay);
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

  // Se envía pulso de habilitación del TRIAC
  if(micros()>pulseOn && pulseOn>0 && pulseOff>0){
    pulseOn = 0;
    if(pulseDelay<(t10mSEG-G_PULSE_WIDTH))    // Envía pulso solo si Output >0
      digitalWrite(RELAY_PIN, HIGH);
  }
  if(micros()>pulseOff && pulseOff>0 && pulseOn==0){
    pulseOff = 0;
    if(pulseDelay>(ZC_PULSE_WIDTH/2))
      digitalWrite(RELAY_PIN, LOW);   // Baja pulso solo si Output <180
  }
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
 
