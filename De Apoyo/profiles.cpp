#include <Arduino.h>
#include "my_profiles.h"

const int MAX_PROFILES = 10; // Máximo número de perfiles
char profileNames[MAX_PROFILES][50]; // Array para almacenar los nombres de los perfiles
struct ActualProfile{
    const char *name;
    int time[15];
    int temperature[15];
    int length;
} ;

ActualProfile myProfile;

void setup() {
    Serial.begin(115200);

    initializeProfiles();

    // Mostrar el número de perfiles cargados
    int profileCount = ReflowHeatingProfileController_getProfileCount(&reflowHeatingProfileController);
    Serial.print("Number of profiles loaded: ");
    Serial.println(profileCount);

    // Guardar los nombres de los perfiles en el array
    for (int i = 0; i < profileCount; i++) {
        const char* profileName = ReflowHeatingProfileController_getProfileName(&reflowHeatingProfileController, i);
        strncpy(profileNames[i], profileName, sizeof(profileNames[i]) - 1);
        profileNames[i][sizeof(profileNames[i]) - 1] = '\0'; // Asegurar que la cadena esté terminada
    }

    // Listar los nombres de los perfiles
    Serial.println("Profile names:");
    for (int i = 0; i < profileCount; i++) {
        Serial.printf("%2d: ", i);
        Serial.println(profileNames[i]);
    }

    // Seleccionar un perfil al azar
    int randomIndex = SN60PB40v2;

    myProfile.name = ReflowHeatingProfileController_getProfileName(&reflowHeatingProfileController, randomIndex);
    
    ReflowHeatingProfileController_selectProfile(&reflowHeatingProfileController, myProfile.name);

    // Mostrar el contenido del perfil seleccionado
    Serial.print("Selected profile: ");
    Serial.println(myProfile.name);
    Serial.println("Profile content:");
    myProfile.length = ReflowHeatingProfileController_getProfileLength(&reflowHeatingProfileController);
    for (int i = 0; i < myProfile.length; i++) {
        myProfile.time[i] = ReflowHeatingProfileController_getTime(&reflowHeatingProfileController, i);
        myProfile.temperature[i] = ReflowHeatingProfileController_getTemperatureForIndex(&reflowHeatingProfileController, i);
        Serial.print("Time: ");
        Serial.print(myProfile.time[i]);
        Serial.print(" Temperature: ");
        Serial.println(myProfile.temperature[i]);
    }
}

void loop() {
    // Tu código principal aquí
}
