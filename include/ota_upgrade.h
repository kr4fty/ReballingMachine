/******************************************************************************
 *
 *  ota_upgrade.h
 *  
 *  Se encarga de realizar las actualizaciones mediante el sistema OTA, una vez
 *  ingresado al modo actualizacion.
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  Autor: Tapia Velasquez Favio
 * 
 *****************************************************************************/

#ifndef OTA_UPGRADE_H
#define OTA_UPGRADE_H

#include <WiFi.h>
#include <ArduinoOTA.h>
#include "display.h"

const char* ssid = "AP-Mode";
const char* password = "elpululo2009";

void startOtaUpgrade()
{
    #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Booting");
    #endif

    lcd.setCursor(0*6*MIDLE_TEXT, 0*8*MIDLE_TEXT);
    lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    lcd.setTextSize(MIDLE_TEXT);
    lcd.println("System update");
    lcd.setTextSize(LITTLE_TEXT);
    lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef DEBUG
    Serial.print(".");
    #endif
    lcd.print(".");
    }
    ArduinoOTA.setPort(3232);
    ArduinoOTA
    .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
        else // U_SPIFFS
        type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        #ifdef DEBUG
        Serial.println("Start updating " + type);
        #endif
        lcd.setCursor(0*6*LITTLE_TEXT, 9*8*LITTLE_TEXT);
        lcd.println("Start updating " + type);
        lcd.print("Progress: ");
    })
    .onEnd([]() {
        #ifdef DEBUG
        Serial.println("\nEnd");
        #endif
        lcd.setTextSize(LITTLE_TEXT);
        lcd.setCursor(0*6*LITTLE_TEXT, 15*8*LITTLE_TEXT);
        lcd.print("End");

        //while(!encoder.isEncoderButtonClicked()); // Esperar a presionar boton para finalizar
    })
    .onProgress([](unsigned int progress, unsigned int total) {
        #ifdef DEBUG
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        #endif
        lcd.setTextSize(LITTLE_TEXT);
        lcd.setCursor(10*6*LITTLE_TEXT, 10*8*LITTLE_TEXT);
        lcd.printf("%u%%",progress / (total / 100));
        lcd.setTextSize(MIDLE_TEXT);
        lcd.setCursor((progress / (total / 14))*5*MIDLE_TEXT, 6*8*MIDLE_TEXT);
        lcd.printf("%c",0xdb); // ascii de █ para la barra de progreso
    })
    .onError([](ota_error_t error) {
        #ifdef DEBUG
        Serial.printf("Error[%u]: ", error);
        #endif
        lcd.printf("Error[%u]: ", error);
        #ifdef DEBUG
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        #endif
    });

    ArduinoOTA.begin();
    #ifdef DEBUG
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    #endif

    lcd.println("\n\n");
    lcd.println("Ready");
    lcd.print("Connected to ");
    lcd.println(ssid);
    lcd.print("IP address: ");
    lcd.println(WiFi.localIP());

    while(1)
    ArduinoOTA.handle();
}

#endif