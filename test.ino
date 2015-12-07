#include <Arduino.h>
#include "enc28j60.h"

static byte myip[] = { 192,168,1,200 };
static byte gwip[] = { 192,168,1,1 };

static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };

byte ENC28J60::buffer[500]; // tcp/ip send and receive buffer

void setup() {
    if(ENC28J60::initialize(sizeof ENC28J60::buffer, mymac, 8) == 0) {
        Serial.print("initialization failed");
    } else {
        Serial.print("else");
    }
}

void loop() {
    ENC28J60::customSend();
}

