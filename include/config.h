/******************************** PINOUT **************************************
*
*                        ESP32 WROOM
*                  ,----------------------,
*                --| 3V3              GND |--          
*                --| EN            GPIO23 |--  TFT_SDA
*                --| GPIO36        GPIO22 |--  ZC_PIN
*                --| GPIO39        GPIO01 |--
*                --| GPIO34        GPIO03 |--
*                --| GPIO35        GPIO21 |--  TH2_CS
*        TFT_DC  --| GPIO32           GND |--
*                --| GPIO33        GPIO19 |--  TH1_DO / TH2_DO
* ENCODER_PIN_A  --| GPIO25        GPIO18 |--  TH1_CLK / TH2_CLK / TFT_SCL
* ENCODER_PIN_B  --| GPIO26        GPIO05 |--  TFT_CS
*                --| GPIO27        GPIO17 |--  RELAY_PIN
*                --| GPIO14        GPIO16 |--  TH1_CS
*                --| GPIO12        GPIO04 |--  BUTTON
*                --| VBATT         GPIO00 |--
*                --| GPIO13        GPIO02 |--
*                --| GPIO09        GPIO15 |--  TFT_RST
*                --| GPIO10        GPIO08 |--
*                --| GPIO11,-----, GPIO07 |--
*                --| 5V    | USB | GPIO06 |--
*                  '______________________'
*
******************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H

// Boton **********************************************************************
#define BUTTON                        27

// Encorder *******************************************************************
#define ENCODER_PIN_A                 25
#define ENCODER_PIN_B                 26

// TERMOCUPLA 1 ***************************************************************
//TH1_DO        GPIO19
#define TH1_CS      16
//TH1_CLK       GPIO18

// TERMOCUPLA 2 ***************************************************************
//TH2_DO        GPIO19
#define TH2_CS      21
//TH2_CLK       GPIO18

// LCD ************************************************************************
//TFT_SCL       GPIO18
//TFT_SDA       GPIO23
#define TFT_CS       5
#define TFT_RST     15
#define TFT_DC      32

// Depuración y  envió de datos por puerto Serial *****************************
//#define DEBUG        1
#define SERIAL_PLOTER  1

// Salida de control del Calefactor *******************************************
#define SALIDA_1     4   // GPIO02 uso como ON/OFF
#define SALIDA_2    17   // GPIO17 uso como PMW
#define HEATER_PIN SALIDA_1
#define COOLER_PIN SALIDA_2
#define RELAY_PIN HEATER_PIN //para mantener la compatibilidad en el código

// Constantes PID *************************************************************
#define CONSTANTES
#ifdef CONSTANTES
//              Actual    Board     Heater      Heater Ring     pidtuner.com 
#define Kp         2.0  //1.570     4.215       1.550           0.95
#define Ki   0.0038989  //0.009636  0.03229     0.01418         0.0064518
#define Kd      9.2996  //0         9.887       0               17.5494

// Pequeño OK       Kp= 0.5, Ki= 0.0038989, Kd= 9.2996
// Grande OK        Kp= 0.95, Ki= 0.0064518, Kd= 17.5494

// Calefactor
#define KP_HEATER       0.95
#define KI_HEATER  0.0064518
#define KD_HEATER    17.5494

// Extractor de calor Inferior
#define KP_COOLER        2.5// 0.1
#define KI_COOLER        0.1// 0.004
#define KD_COOLER        2.5// 0.001
#define COOLER_MIN_ANGLE  80 // Mínimo angulo de disparo el cual el motor se frena 180-150
#define COOLER_MAX_ANGLE 149 // Máximo angulo disparo para velocidad plena 180-50
#endif

// Filtro Digital para la entrada de Temperatura
#define ALPHA    0.250

// Formato del Texto **********************************************************
#define LITTLE_TEXT  1
#define MIDLE_TEXT   2
#define BIG_TEXT     4
#define GIGANT_TEXT  6

// Cruce por Cero *************************************************************
#define WINDOW_INTERRUPT    8 // mili seg, para omitir interrupciones por ruido
#define G_PULSE_WIDTH      15 // micro seg. Ancho del pulso en G del triac
#define ZC_PIN             22 // interrupción externa por el GPIO22
#define WINDOW_1Seg      1000 // milli seconds, 1seg
#define ZC_MAX_ANGLE      180
#define t10mSEG         10000 // micros, 1/100Hz, 2 cruces por cada periodo 2*50Hz
#define ZC_PULSE_WIDTH   1000 // micros, Ancho aproximado de cada pulso de ZC (600 con C=2.7nF)
//#define ZC_INTERRUPT_FILTER // Para evitar falsas interrupciones en ZC_PIN

// Timer **********************************************************************
#define PRESCALER          80 // con 80 me queda bien para el rango utilizado

// Actualizacion del firmware a travez de OTA *********************************
//#define ARDUINO_OTA       1 // Definido automatimente en "Platformio.ini", al
                            // seleccionar entorno "default_envs = wroom_ota",
                            // mediante opcion "-D" del indicador build_flags
                            // build_flags =
	                        //              -D ARDUINO_OTA

#endif