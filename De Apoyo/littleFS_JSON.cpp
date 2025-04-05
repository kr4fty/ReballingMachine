#include <LittleFS.h>
#include <ArduinoJson.h>

void setup() {
  Serial.begin(115200);

  // Inicializar LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  Serial.println("LittleFS mounted successfully");

  // Abrir el archivo JSON
  File file = LittleFS.open("/profile.json", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your JSON document.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  const size_t capacity = JSON_OBJECT_SIZE(10) + JSON_ARRAY_SIZE(13) + 100;
  DynamicJsonDocument doc(capacity);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Extraer la información del perfil de temperatura
  JsonArray profile = doc["profile"];
  for (JsonArray::iterator it = profile.begin(); it != profile.end(); ++it) {
    JsonArray stage = *it;
    int time = stage[0];
    int temp = stage[1];
    Serial.print("Time: ");
    Serial.print(time);
    Serial.print(", Temp: ");
    Serial.println(temp);
  }

  file.close();
}

void loop() {
  // No necesitas nada aquí para este ejemplo
}