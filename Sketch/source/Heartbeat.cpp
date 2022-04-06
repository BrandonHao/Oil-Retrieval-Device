#include "Heartbeat.hpp"

#include <inttypes.h>
#include <Arduino.h>

//Heartbeat interval in milliseconds
#define BEAT_INTERVAL   500

uint32_t time;

void initHeartbeat(){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    time = millis();
}

void heartbeat(){
    if(millis() - time > BEAT_INTERVAL){
        time = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
