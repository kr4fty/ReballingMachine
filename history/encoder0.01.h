#ifndef ENCODER_H
#define ENCODER_H


#include <AiEsp32RotaryEncoder.h>
#include "config.h"

AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(ENCODER_PIN_A,
                                                    ENCODER_PIN_B,
                                                    BUTTON,
                                                    ENCODER_VCC_PIN,
                                                    ENCODER_STEPS,
                                                    ENCODER_CENTRAL_PIN_TO_VCC);
void IRAM_ATTR readEncoderISR(){
	encoder.readEncoder_ISR();
}
static unsigned long    lastTimePressed = 0;
long int                encoderCounter=110,
                        oldEncoderCounter=-1;

void initEncoder(){
    encoder.begin();
    encoder.setup(readEncoderISR);
    bool circleValues =false;
    encoder.setBoundaries(0, 300, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    //encoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
    encoder.setAcceleration(100); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
    encoder.setEncoderValue(encoderCounter);
}
void readEncoder(){
    if (encoder.encoderChanged()){ // Para el control de fase, seteo manual
		encoderCounter = encoder.readEncoder();
	}
}
bool isButtonClicked(){
    return encoder.isEncoderButtonClicked();
}
#endif