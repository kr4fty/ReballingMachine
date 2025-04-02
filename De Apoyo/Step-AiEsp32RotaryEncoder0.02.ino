#include <Arduino.h>
#include "config.h"
#include <SPI.h>
#include <Wire.h>
#include <max6675.h>
#include <AiEsp32RotaryEncoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define WindowSize 200
unsigned long windowStartTime, startTime;

AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(ENCODER_PIN_A, ENCODER_PIN_B, BUTTON, ENCODER_VCC_PIN, ENCODER_STEPS);
void IRAM_ATTR readEncoderISR(){
	encoder.readEncoder_ISR();
}
static unsigned long lastTimePressed = 0;

Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
MAX6675 thermocouple1(TH1_CS);
MAX6675 thermocouple2(TH2_CS);

double    Time;
double    temp1, temp2;
#ifdef ALPHA
double    temp1Filtered, temp2Filtered;
#endif
long int  encoderCounter=90, oldEncoderCounter=-1;

uint32_t  zcNextTime=0, nextTime; 
uint64_t  pulseOn, pulseOff, pulseDelay=0;
bool      zcFlag=false, pulseFlag=false, pulseStatus;
long int  zcCounter=0;

#define   TIMER1 1
#define   TIMER2 2
hw_timer_t *Timer1 = NULL;
hw_timer_t *Timer2 = NULL;
uint16_t  timerTicks;

double    pulseDelayTransformed;

#ifdef ZC_INTERRUPT_FILTER
  uint32_t zcNextTime=0;
  bool zcFlag=false;
#endif

void IRAM_ATTR zc_ISR()
{
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

void IRAM_ATTR Timer1_ISR()
{
    if(timerTicks<(t10mSEG-G_PULSE_WIDTH)) // Si es 0 lo mantengo apagado siempre
        digitalWrite(RELAY_PIN, HIGH);       // => nunca entraría aquí en ese caso
}
void IRAM_ATTR Timer2_ISR()
{
    if(timerTicks>(ZC_PULSE_WIDTH/2)) // Si es 180 lo mantengo encendido siempre
        digitalWrite(RELAY_PIN, LOW);   // => nunca entraría aquí en ese caso
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(ZC_PIN, INPUT);
  
  digitalWrite(RELAY_PIN,LOW);

  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  encoder.begin();
  encoder.setup(readEncoderISR);
  bool circleValues = false;
  encoder.setBoundaries(0, ZC_MAX_ANGLE, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  //encoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
	encoder.setAcceleration(100); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  encoder.setEncoderValue(encoderCounter);

  lcd.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Para sobreescribir puntos
  lcd.fillScreen(ST77XX_BLACK);
  lcd.cp437(true);
  lcd.setRotation(1);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.print("Press Button");
  lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);

  do{
    if (encoder.encoderChanged()){ // Para el control de fase, seteo manual
		  encoderCounter = encoder.readEncoder();
	  }

    if (oldEncoderCounter != encoderCounter) {
      lcd.setCursor(2*6*BIG_TEXT, 2*7*BIG_TEXT);
      lcd.setTextSize(BIG_TEXT);
      lcd.printf("%3d", encoderCounter);
      oldEncoderCounter = encoderCounter;

      // Para obtener la proporción correcta de la superficie del semiciclo
      // La superficie del SENO no cambia linealmente con el angulo
      pulseDelayTransformed = acos(1-encoderCounter/90.0)*(180.0/M_PI);
      pulseDelay = t10mSEG-(uint64_t)map(pulseDelayTransformed, 0, 
                                         ZC_MAX_ANGLE, G_PULSE_WIDTH, 
                                         t10mSEG-ZC_PULSE_WIDTH/2);
    }  
  }while(!encoder.isEncoderButtonClicked());
  
  digitalWrite(LED_BUILTIN, HIGH);

  lcd.fillScreen(ST77XX_BLACK);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.setCursor(0*6*MIDLE_TEXT, 2*7*MIDLE_TEXT);
  lcd.print("Angulo: ");
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  oldEncoderCounter=-1;

  #ifdef ALPHA
  temp1Filtered = thermocouple1.readCelsius();
  temp2Filtered = thermocouple2.readCelsius();
  delay(200);
  #endif
  for(int i=0; i<75; i++){
    temp1= thermocouple1.readCelsius();
    temp2= thermocouple2.readCelsius();
    #ifdef ALPHA
    temp1Filtered = (ALPHA*temp1) + ((1-ALPHA)*temp1Filtered);
    temp2Filtered = (ALPHA*temp2) + ((1-ALPHA)*temp2Filtered);
    #endif
    delay(200);
  }
  
  attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_ISR,RISING);

  Timer1 = timerBegin(TIMER1, PRESCALER, true);
  timerAttachInterrupt(Timer1, &Timer1_ISR, true);
  timerAlarmWrite(Timer1, 1000000, false);

  Timer2 = timerBegin(TIMER2, PRESCALER, true);
  timerAttachInterrupt(Timer2, &Timer2_ISR, true);
  timerAlarmWrite(Timer2, 1000000, false);

  windowStartTime = millis();
  startTime = windowStartTime;
  nextTime=millis() + WINDOW_1Seg;
}

void loop() {
  if (encoder.encoderChanged()){ // Para el control de fase, seteo manual
		encoderCounter = encoder.readEncoder();
	}

  if (oldEncoderCounter != encoderCounter) {
    lcd.setCursor(2*6*BIG_TEXT, 2*7*BIG_TEXT);
    lcd.setTextSize(BIG_TEXT);
    lcd.printf("%3d", encoderCounter);
    oldEncoderCounter = encoderCounter;
    // Para obtener la proporción correcta de la superficie del semiciclo
    // La superficie del SENO no cambia linealmente con el angulo
    pulseDelayTransformed = acos(1-encoderCounter/90.0)*(180.0/M_PI);
    pulseDelay = t10mSEG-(uint64_t)map(pulseDelayTransformed, 0, ZC_MAX_ANGLE, 
                                     G_PULSE_WIDTH, t10mSEG-ZC_PULSE_WIDTH/2);
    lcd.setCursor(0*6*MIDLE_TEXT, 0*7*MIDLE_TEXT);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.printf("%5d", pulseDelay);
  }
  
  /*************************** LEO LAS TEMPERATURAS **************************/
  // Se leen cada 200 mili segundos, max permitida en los modulos MAX6675
  if (millis() > windowStartTime){
    temp1 = thermocouple1.readCelsius();
    temp2 = thermocouple2.readCelsius();

    // Se aplica el filtro pasabajos solo si se selecciona explicitamente
    #ifdef ALPHA
    temp1Filtered = (ALPHA*temp1) + ((1-ALPHA)*temp1Filtered);
    temp1 = temp1Filtered;
    temp2Filtered = (ALPHA*temp2) + ((1-ALPHA)*temp2Filtered);
    temp2 = temp2Filtered;
    #endif
    
    windowStartTime = millis() + WindowSize;
  }
  /***************************************************************************/
  
  /***************************** CONTROL de FASE *****************************/
  // Filtro para falsos positivos
  #ifdef ZC_INTERRUPT_FILTER
  if(millis()>zcNextTime && zcFlag){
    zcFlag = false;
    zcCounter++;
  }
  #endif

  // Calculo de los timerTick para el pulseDelay dado
  // timerTicks = (pulseDelay * 1x10⁻⁶ * 80x10⁶) / PRESCALER = pulseDelay
  timerTicks = (uint16_t)pulseDelay;
  timerAlarmWrite(Timer1, timerTicks, false);
  timerAlarmWrite(Timer2, timerTicks+G_PULSE_WIDTH, false);
  /***************************************************************************/

  /***************************** ENVIO de DATOS ******************************/
  // Cuento interrupsiones de cruce por cero, por segundo, impresion  y envio
  // de datos por puerto serie
  if(millis()>nextTime){
    #ifdef DEBUG
    Time = (millis()-startTime)/1000.0;
    // Para graficar y obtener datos utilizando serial_port_plotter
    Serial.printf("$%d %.2f %.2f %.2f;", encoderCounter, Time, temp1, temp2);
    #endif

    lcd.setCursor(6*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.printf("%3d", zcCounter);
    zcCounter=0;
    nextTime=millis() + WINDOW_1Seg;
  }
  /***************************************************************************/
}