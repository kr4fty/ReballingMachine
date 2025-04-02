#ifndef _ENCODER_H
#define _ENCODER_H

#include <AiEsp32RotaryEncoder.h>
#ifdef __AVR__
#include <digitalWriteFast.h>
#else
#define digitalReadFast digitalRead
#endif
#include "config.h"

#define ENCODER_VCC_PIN               -1 // Using onboard VCC
#define ENCODER_STEPS                  4
#define ENCODER_CENTRAL_PIN_TO_VCC false // middle pin connected to GND

AiEsp32RotaryEncoder myEnc = AiEsp32RotaryEncoder(ENCODER_PIN_A,
    ENCODER_PIN_B,
    BUTTON,
    ENCODER_VCC_PIN,
    ENCODER_STEPS,
    ENCODER_CENTRAL_PIN_TO_VCC);

void readEncoderISR()
{
    myEnc.readEncoder_ISR();
}

void encoder_init()
{
    myEnc.begin();
    myEnc.setup(readEncoderISR);

    bool circleValues =false;
    myEnc.setBoundaries(0, 300, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    //encoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
    myEnc.setAcceleration(150); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
    myEnc.setEncoderValue(0);
}

void encoder_setBasicParameters(long minValue, long maxValue, bool circleValues=false , long newValue=0, long accel=150)
{
    myEnc.setBoundaries(minValue, maxValue, circleValues);
    myEnc.setEncoderValue(newValue);
    myEnc.setAcceleration(accel);
}

void encoder_write(long value)
{
    myEnc.setEncoderValue(value);
}

long encoder_read()
{ 
    return  myEnc.readEncoder();
}

long encoder_encoderChanged()
{
    return myEnc.encoderRotationDetected();
}

/********************************* BOTON *************************************/
enum ClickType {
    NO_CLICK,       // No hay pulsacion
    SHORT_CLICK,    // Pulsacion corta
    LONG_CLICK,     // Pulsacion larga
    DOUBLE_CLICK    // Doble pulsacion
};

bool wasButtonDown    = false; // se presiono el botón
bool checkDoubleClick = false; // si detecta una pulsación rápida es posible que venga una doble pulsación
bool longClickDetected;        // True: se forzó pulsación larga, ignorar click al soltar el botón
uint8_t _clickCounter=0; // 
int8_t lastMovementDirection = 0; // 1 right; -1 left

//paramaters for button
unsigned long shortPressAfterMiliseconds = 250; // Tiempo de espera para una pulsación corta
unsigned long longPressAfterMiliseconds = 1000; // ¿Cuánto tiempo se considera una pulsación larga?
unsigned long lastTimeButtonDown = millis();           // tiempo en el que duro la pulsación
unsigned long lastTimeButtonClick;
unsigned long maxTimeBetween2Events = 300;

bool isButtonDown()
{
    return digitalReadFast(BUTTON)? false : true;
}

bool isButtonUp()
{
    return digitalReadFast(BUTTON)? true : false;
}

// Devuelve el tipo de pulsación detectado. Caso contrario, un NO_CLICK
uint8_t isButtonClicked()
{
    unsigned long timeDiff = millis() - lastTimeButtonDown;
    uint8_t key = NO_CLICK;
    bool currentButtonState = digitalReadFast(BUTTON);
    
    // Boton presionado, button down.
    if (!currentButtonState){
        // Lo hacemos por unica vez en cada ciclo click, button down a button up
        if(!wasButtonDown) {
            wasButtonDown = true;   //  No se ingresara aqui hasta el proximo inicio de click
            lastTimeButtonDown = millis();  // Iniciar el temporizador
            longClickDetected = false;
            //Serial.printf("button down\n");
        }
        // Si el boton permanece presionado, chequeo por pulzacion larga
        else{
            // Considerar como pulsación larga si se mantuvo presionado más de 'longPressAfterMiliseconds'
            if(timeDiff > longPressAfterMiliseconds){
                if(!longClickDetected){ // el boton sigue presionado?
                    key = LONG_CLICK;
                    longClickDetected = true;
                    //Serial.printf("Pulsacion Larga, Duracion: %d, Key:%d\n", timeDiff, key);
                } 
                // se ha detectado una pulsacion larga, por lo que no tiene sentido ahora preguntar por pulsacion corta
                return key;
            }
        }
    }
    // Boton liberado, se hizo click
    else{
        if(longClickDetected){ // Si hubo una pulsacion larga, reseteo valores para ignorar el click al soltar el boton
            longClickDetected = false;
            wasButtonDown = false;
        }
        else if (wasButtonDown){
            //Serial.printf("button up\n");
            _clickCounter++;
            //Serial.printf("Clicks = %d Time: %d ", _clickCounter, timeDiff);
            wasButtonDown = false;

            if (timeDiff > shortPressAfterMiliseconds){ // Se detecto pulsacion corta
                //Serial.printf("\n");
            } else { // Se detecto pulsacion corta y rapida. Posible intento de doble click?
                checkDoubleClick = true;
                //Serial.printf("Fast click\n");
            }
        }
    }
    // Pasado un tiempo maxTimeBetween2Events, veo si fue un click corto o un doble click rapido
    if(timeDiff>maxTimeBetween2Events && _clickCounter){
        if(_clickCounter==1){ // Solo se detecto una pulsacion, entonces es una pulsacion corta
            key = SHORT_CLICK;
            //Serial.printf("Pulsacion Corta\n");

        }
        else if(_clickCounter>1){ // Se detecto varias pulsaciones, entonces es un doble click (podria haber 3 o mas :P)
            if(checkDoubleClick){
                key = DOUBLE_CLICK;
                //Serial.printf("Doble Pulsacion\n");
            }
        }
        
        _clickCounter = 0;
        checkDoubleClick = false;
    }

    return key;
}

#endif