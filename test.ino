#include <Arduino.h>
#include "enc28j60.h"



void setup() {

}

void loop() {
    if (packetLoop(packetReceive())) {
        packetWriteToConsole(sizeof page - 1);
        //packetTcpSend(size) //make_tcp_ack....
    }
}