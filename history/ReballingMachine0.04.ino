#include <Arduino.h>
#include "config.h"
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <max6675.h>
#include <AiEsp32RotaryEncoder.h>

AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(ENCODER_PIN_A, ENCODER_PIN_B, BUTTON, ENCODER_VCC_PIN, ENCODER_STEPS);
void IRAM_ATTR readEncoderISR(){
	encoder.readEncoder_ISR();
}
static unsigned long lastTimePressed = 0;
long int encoderCounter=110, oldEncoderCounter=-1;

uint32_t zcNextTime=0, nextTime; 
uint64_t pulseOn, pulseOff, pulseDelay=0;
bool zcFlag=false, pulseFlag=false, pulseStatus;
long int zcCounter=0;

void IRAM_ATTR zc_isr() {
  // Filtro para falsos positivos
  #ifdef ZC_INTERRUPT_FILTER
  if(!zcFlag){
    zcFlag = true;
    zcNextTime=millis()+WINDOW_INTERRUPT;
    pulseFlag = true;
    pulseOn = micros()+pulseDelay;
  }
  #else
    zcCounter++;
    pulseFlag = true;
    pulseOn = micros()+pulseDelay;
  #endif
}

Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

MAX6675 thermocouple(TH1_CS);

#define WindowSize 200
unsigned long windowStartTime;
//DEBUG
#ifdef DEBUG
unsigned long startTime;
#endif

double Setpoint, Input, Output;
PID myPID = PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
 
// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(ZC_PIN, INPUT);

  digitalWrite(RELAY_PIN,LOW);

  //Setpoint = 120;
  //turn the PID on
  myPID.SetSampleTime(25);
  myPID.SetOutputLimits(0, ZC_MAX_ANGLE);
  myPID.SetMode(AUTOMATIC);

   // Use this initializer if using a 1.8" TFT screen:
  lcd.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Para sobre escribir puntos
  lcd.fillScreen(ST77XX_BLACK);
  lcd.setRotation(1);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.setCursor(0, 0);
  lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  lcd.print("Press key");

  // Encoder
  //encoder.areEncoderPinsPulldownforEsp32 = false;
  encoder.begin();
  encoder.setup(readEncoderISR);
  bool circleValues = true;
  encoder.setBoundaries(0, 300, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  //encoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
	encoder.setAcceleration(250); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  encoder.setEncoderValue(encoderCounter);

  uint8_t clicks = 0;
  while(clicks < 2){
    if (encoder.isEncoderButtonClicked() && (millis() - lastTimePressed >= 100)){
        lastTimePressed = millis();
        clicks ++;
    }      
  }

  lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
  lcd.print("Temp   Out");
  lcd.setCursor(4*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
  lcd.print("Pulsos");
  lcd.setCursor(3*6*MIDLE_TEXT, 4*8*MIDLE_TEXT);
  lcd.print("Setpoint");
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_isr,RISING);
  nextTime=millis() + WINDOW_1Seg;

  // DEBUG
  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  windowStartTime = millis();
  //DEBUG
  #ifdef DEBUG
  startTime = windowStartTime;
  #endif
}

// the loop function runs over and over again forever

double oldInput=0;
#ifdef ALPHA
double InputFiltered;
#endif
void loop() {
  //FILTRO DIGITAL
  #ifdef ALPHA
  InputFiltered = thermocouple.readCelsius();

  while(1){
  #endif
    // Seteo del Setpoint
    if (encoder.encoderChanged()){ // Para el control de fase, seteo manual
      encoderCounter = encoder.readEncoder();
    }
    if (oldEncoderCounter != encoderCounter) {
      lcd.setCursor(2*6*BIG_TEXT, 3*7*BIG_TEXT);
      lcd.setTextSize(BIG_TEXT);
      lcd.printf("%3d", encoderCounter);
      lcd.setTextSize(MIDLE_TEXT);
      Setpoint = encoderCounter;
      oldEncoderCounter = encoderCounter;
    }

    //Senso la temperatura cada 200mSeg (frecuencia maxima a la que lee el sensor max6675)
    if (millis() > windowStartTime){
      Input = thermocouple.readCelsius();
      #ifdef ALPHA
      InputFiltered = (ALPHA*Input) + ((1-ALPHA)*InputFiltered);
      Input = InputFiltered;
      #endif

      // DEBUG
      /*#ifdef DEBUG
      //double Time = (millis()-startTime)/1000.0;
      Serial.print(Input);
      Serial.print(" ");
      Serial.println(0);
      #endif*/

      windowStartTime = millis() + WindowSize;
    }

    myPID.Compute();

    /***************************** CONTROL DE FASE ****************************/
    pulseDelay = t10mSEG-map(Output, 0, ZC_MAX_ANGLE, G_PULSE_WIDTH, t10mSEG-ZC_PULSE_WIDTH/2);

    // Filtro para falsos positivos
    #ifdef ZC_INTERRUPT_FILTER
    if(millis()>zcNextTime && zcFlag){
      zcFlag = false;
      zcCounter++;
    }
    #endif

    // Se envia pulso de habilitacion del TRIAC
    if(micros()>pulseOn && pulseFlag && Output>0){
      pulseOff = micros() + G_PULSE_WIDTH ;
      pulseStatus = HIGH;
      digitalWrite(RELAY_PIN, pulseStatus);
      pulseFlag = false;
    }
    if(micros()>pulseOff && !pulseFlag && pulseStatus){
      pulseStatus = LOW;
      digitalWrite(RELAY_PIN, pulseStatus);    
    }
    /*************************************************************************/

    if(millis()>nextTime){

      if(oldInput!=Input){
        lcd.setCursor(0*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
        lcd.printf("%3d",(uint16_t)Input);

        oldInput = Input;
      }
      lcd.setCursor(6*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
      lcd.printf("%4d",pulseDelay);
      lcd.setCursor(7*6*MIDLE_TEXT, 3*8*MIDLE_TEXT);
      lcd.printf("%3d",zcCounter);
      zcCounter=0;

      // DEBUG
      #ifdef DEBUG
      Serial.print(Input);
      Serial.print(" ");
      Serial.println(Output);
      #endif

      nextTime=millis() + WINDOW_1Seg;      
    }
  //FILTRO DIGITAL
  #ifdef ALPHA
  }
  #endif
}
