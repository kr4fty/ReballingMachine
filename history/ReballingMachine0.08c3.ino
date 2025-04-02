#include <Arduino.h>
#include "config.h"
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <max6675.h>
#include <AiEsp32RotaryEncoder.h>

/********************************* OTA *************************************/
const char* ssid = "AP-Mode";
const char* password = "elpululo2009";
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

AsyncWebServer server(80);
unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}
/******************************* END OTA ***********************************/

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
    if(timerTicks<(t10mSEG-G_PULSE_WIDTH)) // Si es 0 lo mantengo apagado siempre
      digitalWrite(RELAY_PIN, HIGH);       // => nunca entraría aquí en ese caso
}
void IRAM_ATTR Timer2_ISR()
{
  if(timerTicks>(ZC_PULSE_WIDTH/2)) // Si es 180 lo mantengo encendido siempre
    digitalWrite(RELAY_PIN, LOW);   // => nunca entraría aquí en ese caso
}

static unsigned long lastTimePressed = 0;
long int encoderCounter=110, oldEncoderCounter=-1;

#ifdef ZC_INTERRUPT_FILTER
  uint32_t zcNextTime=0;
  bool zcFlag=false;
#endif
uint32_t nextTime; 
uint64_t pulseDelay=0;
long int zcCounter=0;
double pulseDelayTransformed;

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

Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

MAX6675 thermocouple1(TH1_CS);  // Heater
MAX6675 thermocouple2(TH2_CS);  // Board

#define WindowSize 200
unsigned long windowStartTime;
//DEBUG
#ifdef DEBUG
unsigned long startTime;
#endif
double oldInput1=0, oldInput2=0;
#ifdef ALPHA
double InputFiltered1, InputFiltered2;
#endif

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
PID myPID = PID(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
 
// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(ZC_PIN, INPUT);

  digitalWrite(RELAY_PIN,LOW);

  /********************************* OTA *************************************/
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is ElegantOTA AsyncDemo.");
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.println("HTTP server started");
  /******************************* END OTA ***********************************/

  //Setpoint1 = 120;
  //turn the PID on
  myPID.SetSampleTime(WindowSize);
  myPID.SetOutputLimits(0, ZC_MAX_ANGLE);
  myPID.SetMode(AUTOMATIC);

   // Use this initializer if using a 1.8" TFT screen:
  lcd.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Para sobre escribir puntos
  lcd.fillScreen(ST77XX_BLACK);
  lcd.setRotation(1);
  lcd.setTextSize(MIDLE_TEXT);
  lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
  lcd.print("Setpoint:");
  lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
  lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  lcd.print("Press Button");

  // Encoder
  //encoder.areEncoderPinsPulldownforEsp32 = false;
  encoder.begin();
  encoder.setup(readEncoderISR);
  bool circleValues =false;
  encoder.setBoundaries(0, 300, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  //encoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
	encoder.setAcceleration(100); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  encoder.setEncoderValue(encoderCounter);

  windowStartTime = millis();
  do{
    if (encoder.encoderChanged()){ // Para el control de fase, seteo manual
		  encoderCounter = encoder.readEncoder();
	  }

    if (oldEncoderCounter != encoderCounter) {
      lcd.setCursor(2*6*BIG_TEXT, 2*7*BIG_TEXT);
      lcd.setTextSize(BIG_TEXT);
      lcd.printf("%3d", encoderCounter);
      oldEncoderCounter = encoderCounter;
      pulseDelay = t10mSEG-map(encoderCounter, 0, ZC_MAX_ANGLE, G_PULSE_WIDTH, t10mSEG-ZC_PULSE_WIDTH/2);
    }
    if (millis() > windowStartTime){
      Input1 = thermocouple1.readCelsius();
      Input2 = thermocouple2.readCelsius();
      #ifdef ALPHA
      InputFiltered1 = (ALPHA*Input1) + ((1-ALPHA)*InputFiltered1);
      Input1 = InputFiltered1;
      InputFiltered2 = (ALPHA*Input2) + ((1-ALPHA)*InputFiltered2);
      Input2 = InputFiltered2;
      #endif

      windowStartTime = millis() + WindowSize;
    }
  }while(!encoder.isEncoderButtonClicked());

  lcd.setTextSize(MIDLE_TEXT);
  lcd.fillScreen(ST77XX_BLACK);
  lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
  lcd.print("Heater    Out");
  lcd.setCursor(0*6*MIDLE_TEXT, 2*8*MIDLE_TEXT);
  lcd.print("Board  Pulsos");
  lcd.setCursor(3*6*MIDLE_TEXT, 4*8*MIDLE_TEXT);
  lcd.print("Setpoint");
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  oldEncoderCounter=-1;

  attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_isr,RISING);
  nextTime=millis() + WINDOW_1Seg;

  Timer1 = timerBegin(TIMER1, PRESCALER, true);
  timerAttachInterrupt(Timer1, &Timer1_ISR, true);
  timerAlarmWrite(Timer1, 1000000, false);

  Timer2 = timerBegin(TIMER2, PRESCALER, true);
  timerAttachInterrupt(Timer2, &Timer2_ISR, true);
  timerAlarmWrite(Timer2, 1000000, false);


  windowStartTime = millis();
  //DEBUG
  #ifdef DEBUG
  Serial.begin(9600);
  startTime = windowStartTime;
  #endif
}

// the loop function runs over and over again forever
void loop() {
  /********************************* OTA *************************************/
  ElegantOTA.loop();
  /******************************* END OTA ***********************************/

  // Seteo del Setpoint
  if (encoder.encoderChanged()){ // Para el control de fase, seteo manual
    encoderCounter = encoder.readEncoder();
  }
  if (oldEncoderCounter != encoderCounter) {
    lcd.setCursor(2*6*BIG_TEXT, 3*7*BIG_TEXT);
    lcd.setTextSize(BIG_TEXT);
    lcd.printf("%3d", encoderCounter);
    lcd.setTextSize(MIDLE_TEXT);
    Setpoint1 = encoderCounter;
    oldEncoderCounter = encoderCounter;
  }

  //Senso la temperatura cada 200mSeg (frecuencia máxima a la que lee el sensor max6675)
  if (millis() > windowStartTime){
    Input1 = thermocouple1.readCelsius();
    Input2 = thermocouple2.readCelsius();
    #ifdef ALPHA
    InputFiltered1 = (ALPHA*Input1) + ((1-ALPHA)*InputFiltered1);
    Input1 = InputFiltered1;
    InputFiltered2 = (ALPHA*Input2) + ((1-ALPHA)*InputFiltered2);
    Input2 = InputFiltered2;
    #endif

    // DEBUG
    #ifdef DEBUG
    //double Time = (millis()-startTime)/1000.0;
    //Serial.print(Input1);
    //Serial.print(" ");
    //Serial.println(0);
    #ifdef ALPHA
    //Serial.printf("$%.2f %.2f %.2f;",Input1,InputFiltered1,Output1);
    #else
    //Serial.printf("$%.2f %.2f;",Input1,Output1);
    #endif
    #endif

    windowStartTime = millis() + WindowSize;
  }

  myPID.Compute();

  /***************************** CONTROL DE FASE ****************************/
  // Para obtener la proporción correcta de la superficie del semiciclo
  // La superficie del SENO no cambia linealmente con el angulo
  pulseDelayTransformed = acos(1-Output1/90.0)*(180.0/M_PI);
                                                             
  pulseDelay = t10mSEG-(uint64_t)map(pulseDelayTransformed, 0, ZC_MAX_ANGLE, G_PULSE_WIDTH, t10mSEG-ZC_PULSE_WIDTH/2);

  // Calculo de los timerTick para el pulseDelay dado
  // timerTicks = (pulseDelay * 1x10⁻⁶ * 80x10⁶) / PRESCALER = pulseDelay
  timerTicks = (uint16_t)pulseDelay;
  timerAlarmWrite(Timer1, timerTicks, false);
  timerAlarmWrite(Timer2, timerTicks+G_PULSE_WIDTH, false);

  // Filtro para falsos positivos
  #ifdef ZC_INTERRUPT_FILTER
  if(millis()>zcNextTime && zcFlag){
    zcFlag = false;
    zcCounter++;
  }
  #endif
  /*************************************************************************/

  if(millis()>nextTime){

    if(oldInput1!=Input1 || oldInput2!=Input2){
      lcd.setCursor(0*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
      lcd.printf("%3d",(uint16_t)Input1);
      lcd.setCursor(0*6*MIDLE_TEXT, 3*8*MIDLE_TEXT);
      lcd.printf("%3d",(uint16_t)Input2);

      oldInput1 = Input1;
      oldInput2 = Input2;
    }
    lcd.setCursor(9*6*MIDLE_TEXT, 1*8*MIDLE_TEXT);
    lcd.printf("%4d",(uint8_t)Output1);
    lcd.setCursor(10*6*MIDLE_TEXT, 3*8*MIDLE_TEXT);
    lcd.printf("%3d",zcCounter);
    zcCounter=0;

    // DEBUG
    #ifdef DEBUG
    Serial.printf("$%.2f %.2f %d;",Input1, Input2, (uint16_t)Output1);
    #endif

    nextTime=millis() + WINDOW_1Seg;      
  }
}
