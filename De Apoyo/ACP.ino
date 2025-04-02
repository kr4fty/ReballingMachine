//***************************************************************
// Desarrollado por Garikoitz Martínez [garikoitz.info] [05/2020]
// https://garikoitz.info/blog/?p=638
// https://garikoitz.info/blog/?p=674
//
// FUENTE: https://garikoitz.info/blog/2020/05/sintonizar-pid-con-arduino-i/
//***************************************************************
//===============================================================
// Librerías
//===============================================================
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <max6675.h>
#include <math.h>
#include <PID_v1.h>
#include "config.h"
//===============================================================
// Variables globales & Constantes
//===============================================================

MAX6675 sensors(TH1_CS);

double Setpoint, Input, Output;
double Kp=55, Ki=29, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);
boolean modo = false; //false=auto true=manual
String modos = "";
float OP;
const int pot =0; // Potenciómetro conectado al pin A0
const int vent =5; // ventilador conectado al pin 5
int velocidad;

Adafruit_ST7735 lcd = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
//===============================================================
// SETUP
//===============================================================
void setup(void) {
    Serial.begin(9600);
    pinMode(vent, OUTPUT);
    
    lcd.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
    lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Para sobreescribir puntos
    lcd.fillScreen(ST77XX_BLACK);
    lcd.setRotation(1);
    lcd.setTextSize(1);
}
//===============================================================
// BUCLE PRINCIPAL
//===============================================================
void loop() {
      // Interruptor Auto/Manual
      if (digitalRead(12) == HIGH){
        modo=true;
        modos="MANUAL";
        myPID.SetMode(MANUAL);
      }else{
          modo=false;
          modos="AUTOMATICO";
          myPID.SetMode(AUTOMATIC);
       }
       
       Input = sensors.readCelsius();
     // MODO MANUAL Potenciómetro ---> PWM Ventilador
     if (modo==true){
         velocidad = analogRead (pot);
         OP = map(velocidad, 0, 1023, 0, 255);
         analogWrite(vent, OP);
         myPID.SetMode(MANUAL);
         //LCD en MANUAL
         lcd.fillScreen(ST77XX_BLACK);
         lcd.setCursor(0,0);
         lcd.print("====== ");
         lcd.print(modos);
         lcd.print(" ======");
         lcd.setCursor(0,1);
         lcd.print("  T: ");             
         lcd.print(Input,2);
         lcd.print("\337C ");
         lcd.setCursor(0,2);
         lcd.print(" OP: "); 
         lcd.print(OP*0.3921);
         lcd.print(" % ");
         lcd.setCursor(0,3);
         lcd.print("PWM: ");
         lcd.print(OP,0);
        // DEBUG PUERTO SERIE (Para Arduino COM Plotter)
        Serial.print("#");            //Char inicio
        Serial.print(0);              //Dejo vacío para ACP
        Serial.write(" ");            //Char separador
        Serial.print(Input,2);        //PV
        Serial.write(" ");            //Char separador
        Serial.print(OP*0.3921,0);    //OP
        Serial.println();
     }
     // MODO AUTOMÁTICO---------------------------------------------
     if (modo==false){
        float set=analogRead(pot);
        Setpoint=map(set,0,1023,60.0,40.0); 
        Input = sensors.readCelsius();
        myPID.SetMode(AUTOMATIC);
        //-----------------------------
        //LCD en AUTO
        lcd.fillScreen(ST77XX_BLACK);
        lcd.setCursor(0,0);
        lcd.print("==== ");
        lcd.print(modos);
        lcd.print(" ====");
        //
        lcd.setCursor(0,1*9);
        lcd.print("PV: ");
        lcd.print(Input,2);
        lcd.print(" \337C ");
        lcd.setCursor(0,2*9);
        lcd.print("SP: ");
        lcd.print(Setpoint,2);
        lcd.print(" \337C ");
        lcd.setCursor(0,3*9);
        lcd.print("OP: ");
        lcd.print(Output*0.3921,2);
        lcd.print(" %");
        //
        myPID.Compute();
        analogWrite(vent, Output);
       //===============================================================
       // DEBUG PUERTO SERIE (Para Arduino COM Plotter)
       //===============================================================
        Serial.print("#");              //Char inicio
        Serial.print(Setpoint,0);       //SP
        Serial.write(" ");              //Char separador
        Serial.print(Input,2);          //PV
        Serial.write(" ");              //Char separador
        Serial.print(Output*0.3921,0);  //OP
        Serial.println();
    }
    delay(1000);
}//Fin void loop