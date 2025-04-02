#ifndef THERMOCOUPLES_H
#define THERMOCOUPLES_H

#include <max6675.h>
#include "config.h"

MAX6675 thermocouple1(TH1_CS);  // Heater
MAX6675 thermocouple2(TH2_CS);  // Board

double oldInput1=0, oldInput2=0;
#ifdef ALPHA
double InputFiltered1, InputFiltered2;
#endif

void readThermocouples(double* input1, double* input2){
    *input1 = thermocouple1.readCelsius();
    *input2 = thermocouple2.readCelsius();
    #ifdef ALPHA
    InputFiltered1 = (ALPHA*(*input1)) + ((1-ALPHA)*InputFiltered1);
    *input1 = InputFiltered1;
    InputFiltered2 = (ALPHA*(*input2)) + ((1-ALPHA)*InputFiltered2);
    *input2 = InputFiltered2;
    #endif
}

#endif