/** Curva perfil de temperatura ***********************************************

 Temp^
     |
     |
TMax_|                                   __.`.__
     |                                 /         \
     |                                /           \
     |            ___________________/             \
     |           /                                  \
     |          /                                    \
     |         /                                      \ 
     |        /                                        \
Tamb_| ______/                                          \_____________
     |
     '------------------------------------------------------------------->
             |   |                   |  |   |    |      |                  Time
     0      t0  t1                  t2 t3  tmax  t4     t5


   Tiempo  Temperaturas    Duración                    Descripción
   ======  ============    ========                    ===========
   t0-t1   1ºC/seg.        Hasta los 150ºC             Pre calentamiento
   t1-t2   0.5ºC/seg.      hasta los 180º, 60 seg Max  Activación del FLUX
   t2-t3   180º a Tmax     45seg MAX                   Reflow / Extracción
   t4-t5   -1ºC/seg.       Tmax a Tamb                 Enfriamiento


 Tmax va a variar de acuerdo a cada placa (disipador, espesor de placa, IC,
 tipo de estaño, etc.). Por lo general integrados con plomo, hasta 183ºC. Para
 ICs sin plomo, hasta unos 218-230ºC


 
  Tiempo[s]	sn60pb40v2	sn63pb37	sn42bi576ag04	sn965ag30cu05	Sn60Pb40v3
  ----------------------------------------------------------------------------
        0		 30			 30			 30				 30				 30
       20					 90											
       30					100											
       40					110											
       90		140						 90				150				120
      110					140											
      120					150											
      130					160											
      150					183											
      180		165						130				175				150
      200					230											
      210		190			235			138				217				180
      220					230											
      240		210			183			165				249				220
      270		190						138				217				220
      330		 50										 50				 50
      340					 50											
      360																
      390								 50								
  
 *****************************************************************************/


#ifndef MY_PROFILES_H
#define MY_PROFILES_H
 
#include "ReflowProfile.h"

#define SN60PB40v1      0
#define SN60PB40v2      1
#define SN60PB40v3      2
#define SN63PB37        3
#define SN42BI576AG04   4
#define SN965AG30CU05   5

float tempSlope[20]; // Pendiente de temperatura
#define MAX_PROFILES 10 // Máximo número de perfiles
char profileNames[MAX_PROFILES][50]; // Array para almacenar los nombres de los perfiles
uint8_t profileCount; //Numero de perfiles
uint8_t profileSelectedIndex; // Perfile Seleccionado

struct ActualProfile{
    const char *name;
    uint16_t time[15]; // Tiempos del perfil seleccionado
    uint16_t temperature[15]; // Setpoints dentro del perfil seleccionado
    uint8_t length; // Paso dentro del perfil seleccionado
} myProfile;
 
ReflowHeatingProfileController reflowHeatingProfileController;
 
void profile_initializeProfiles()
{
    for(uint8_t i=0; i<20; i++){
        tempSlope[0] = 0;
    }
    // Inicializar el controlador
    ReflowHeatingProfileController_init(&reflowHeatingProfileController);

    // Definir los perfiles de calentamiento
    ReflowProfile sn60pb40v1 = {
        {0, 90, 180, 200, 220, 370},
        {30, 130, 150, 217, 217, 0},
        6
    };

    ReflowProfile sn60pb40v2 = {
        {0, 90, 180, 210, 240, 270, 330},
        {30, 140, 165, 190, 210, 190, 50},
        7
    };

    ReflowProfile sn60pb40v3 = {
        {0, 90, 180, 210, 240, 270, 330},
        {30, 120, 150, 180, 220, 120, 50},
        7
    };

    // eutectic SnPb
    ReflowProfile sn63pb37 = {
        {0, 20, 30, 40, 110, 120, 130, 150, 200, 210, 220, 240, 340},
        {30, 90, 100, 110, 140, 150, 160, 183, 230, 235, 230, 183, 50},
        13
    };

    ReflowProfile sn42bi576ag04 = {
        {0, 90, 180, 210, 240, 270, 390},
        {30, 90, 130, 138, 165, 138, 50},
        7
    };

    ReflowProfile sn965ag30cu05 = {
        {0, 90, 180, 210, 240, 270, 330},
        {30, 150, 175, 217, 249, 217, 50},
        7
    };

    ReflowHeatingProfileController_addProfile(&reflowHeatingProfileController, "sn60pb40v1", sn60pb40v1);
    ReflowHeatingProfileController_addProfile(&reflowHeatingProfileController, "sn60pb40v2", sn60pb40v2);
    ReflowHeatingProfileController_addProfile(&reflowHeatingProfileController, "sn60pb40v3", sn60pb40v3);
    ReflowHeatingProfileController_addProfile(&reflowHeatingProfileController, "sn63pb37", sn63pb37);
    ReflowHeatingProfileController_addProfile(&reflowHeatingProfileController, "sn42bi576ag04", sn42bi576ag04);
    ReflowHeatingProfileController_addProfile(&reflowHeatingProfileController, "sn965ag30cu05", sn965ag30cu05);

    // Obtengo el numero de perfiles Guardados
    profileCount = ReflowHeatingProfileController_getProfileCount(&reflowHeatingProfileController);
    /*Serial.print("Number of profiles loaded: ");
    Serial.println(profileCount);*/

    // Guardar los nombres de los perfiles en el array
    for (uint8_t i = 0; i < profileCount ; i++) {
        const char* profileName = ReflowHeatingProfileController_getProfileName(&reflowHeatingProfileController, i);
        strncpy(profileNames[i], profileName, sizeof(profileNames[i]) - 1);
        profileNames[i][sizeof(profileNames[i]) - 1] = '\0'; // Asegurar que la cadena esté terminada
    }
    // Listar los nombres de los perfiles
    /*Serial.println("Profile names:");
    for (int i = 0; i < profileCount; i++) {
        Serial.printf("%2d: ", i);
        Serial.println(profileNames[i]);
    }*/

}

void profile_selectProfile(uint8_t profileSelected)
{
    // Marco el Perfil seleccionado como activo
    ReflowHeatingProfileController_selectProfile(&reflowHeatingProfileController, profileSelected);

    //Guardo los valores del perfil seleccionado
    //Serial.print("Selected profile: ");
    myProfile.name = ReflowHeatingProfileController_getSelectedProfileName(&reflowHeatingProfileController);
    //Serial.println(myProfile.name);
    //Serial.println("Profile content:");
    myProfile.length = ReflowHeatingProfileController_getProfileLength(&reflowHeatingProfileController);
    for (uint8_t i = 0; i < myProfile.length; i++) {
        myProfile.time[i] = ReflowHeatingProfileController_getTime(&reflowHeatingProfileController, i);
        myProfile.temperature[i] = ReflowHeatingProfileController_getTemperatureForIndex(&reflowHeatingProfileController, i);
        /*Serial.print("Time: ");
        Serial.print(myProfile.time[i]);
        Serial.print(" Temperature: ");
        Serial.println(myProfile.temperature[i]);*/
    }
}

void profile_preCalculate(float initialTemp, uint16_t initialTime)
{
    // Cargo los tiempo inicial en el que arranco el sistema
    myProfile.temperature[0] = initialTemp;
    myProfile.time[0] = initialTime;

    // Calculo las pendientes
    for(uint8_t i=0; i<myProfile.length-1; i++){
        tempSlope[i] = (float)(myProfile.temperature[i+1]-myProfile.temperature[i])/(myProfile.time[i+1]-myProfile.time[i]);
    }
}
 
#endif