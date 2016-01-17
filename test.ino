    #include <Arduino.h>
    #include "enc28j60.h"


//    static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };
    static byte srcAddr[] = {0x20, 0x89, 0x84, 0x1F, 0x61, 0x5D};
    static byte destAddr[] = {0x74, 0x69, 0x69, 0x2D, 0x30, 0x31};
    static byte srcV6[] = {0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4, 0x54, 0xBE};
    static byte destV6[] = {0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4, 0x54, 0xBD};
    static byte sendPort[] = {0x0B, 0xB8};
    static byte receivePort[] = {0x0B, 0xB8};

    byte ENC28J60::buffer[500]; // tcp/ip send and receive buffer

    void setup() {
        Serial.begin(19200);
        if(ENC28J60::customInitialize(sizeof ENC28J60::buffer, srcAddr) == 0) {
            Serial.print("initialization failed");
        } else {
            Serial.print("else");
        }
        ENC28J60::createFrame(srcAddr,destAddr,srcV6,destV6,sendPort,receivePort);
    }

    void loop() {
    //    ENC28J60::customSend(srcAddr,destAddr,srcV6,destV6,sendPort,receivePort);
//        ENC28J60::customSend(false, false, false);
        ENC28J60::sendTestFrame();
        delay(1000);
        uint16_t len = ENC28J60::customReceive();
        ENC28J60::readPacket(len);
        delay(1000);
    }



