#include <Arduino.h>
#include "Frame.h"
Frame::Frame(const byte *_destAddr, const  byte *_srcAddr, const  byte *_srcV6,
             const byte *_destV6, const byte  *_sendPort, byte *_receivePort) {
    srcAddr = _srcAddr;
    destAddr = _destAddr;
    srcV6 = _srcV6;
    destV6 = _destV6;
    sendPort = _sendPort;
    receivePort = _receivePort;
}

Frame::~Frame() {
    delete[] srcAddr;
    delete[] destAddr;
    delete[] srcV6;
    delete[] destV6;
    delete[] sendPort;
    delete[] receivePort;
}

const byte * Frame::getSrcAddr() {
    return srcAddr;
}

const byte * Frame::getDestAddr() {
    return destAddr;
}

const  byte* Frame::getSrcV6() {
    return srcV6;
}

const byte* Frame::getDestV6() {
    return destV6;
}

const  byte * Frame::getSendPort() {
    return sendPort;
}

byte * Frame::getReceivePort() {
    return receivePort;
}

uint16_t Frame::getSize() {
    return size;
}
//
//void Frame::setSrcAddr(byte * addr) {
//    delete[] srcAddr;
//    srcAddr = addr;
//}
//
//void Frame::setDestAddr(byte * addr) {
//    delete[] destAddr;
//    destAddr = addr;
//}
//
//void Frame::setSrcV6(byte * addr) {
//    delete[] srcV6;
//    srcV6 = addr;
//}
//
//void Frame::setDestV6(byte * addr) {
//    delete[] destV6;
//    destV6 = addr;
//}
//
//void Frame::setSendPort(byte * port) {
//    delete[] sendPort;
//    sendPort = port;
//}

void Frame::setReceivePort(byte * port) {
    delete[] receivePort;
    receivePort = port;
}

byte * Frame::getTCPPacket( byte* data, const bool ifSyn, const bool ifAck, const bool ifRes,
                            const bool ifFin,  const uint16_t sizeOfData) {

    size = 74 + sizeOfData;
    byte* packet = new byte[size];

    static byte frameTemplate[]{
            //MAC addresses
            0x86,
            0xdd,
            0x60, // version +
            0x00, // traffic class +
            0x00, //
            0x01, // flow control
            0x00, // payload 1st byte
            0x14, // payload 2nd byte (set to 20 as default = tcp header length with no data)
            0x06, // next header (06 = tcp header)
            0x40, // hop limit = 64
            //IPv6 addresses 54
            //ports
            0x00, 0x00, 0x00, 0x02,//Sequence number, theoretically random
            0x00, 0x00, 0x00, 0x00,//Acknowledgment, doesn't matter with SYN (right?)
            0b01010000,
            0b00000000,
            0x05, 0x78  ,
            0x00, 0x00, //checksum
            0x00, 0x00 //URG not set, so value here doesn't matter
//            0x02, 0x04, 0x05, 0x00
    };

    for (int k = 0; k < 6; k++) {
        packet[k] = destAddr[k];
    }
    //destination address
    for (int k = 0; k < 6; k++) {
        packet[k+6] =  srcAddr[k];
    }
    //ipframe
    for (int k = 0; k < 10; k++) {
        packet[k+12] = frameTemplate[k];
    }

    //set payload
    if(sizeOfData > 0) {
        uint16_t payload = sizeOfData + 20;
        packet[18] = (uint8_t) (payload >> 8);
        packet[19] = (uint8_t) (payload);
    }

    //IPv6 Source Address
    for (int k = 0; k < 16; k++) {
        packet[k+22] = srcV6[k];
    }
    //IPv6 Receive Address
    for (int k = 0; k < 16; k++) {
        packet[k+38] = destV6[k];
    }
    //source port
    /*receivePort[0] = destPort[0];
    receivePort[1] = destPort[1];*/
    packet[54] = sendPort[0];
    packet[55] = sendPort[1];
    packet[56] = receivePort[0];
    packet[57] = receivePort[1];

    //rest of frame
    for (int k = 0; k < 16; k++) {
        packet[k+58] = frameTemplate[k+10];
    }

    for(int k = 0; k < size; k++) {
        packet[k + 74] = data[k];
    }

    if(ifSyn) {
        packet[67] = packet[67] | (0b00000010);
    }
    if(ifAck) {
        packet[67] = packet[67] | (0b00010000);
    }
    if(ifRes) {
        packet[67] = 0b00010100;
    }
    if(ifFin) {
        packet[67] = 0b00010001;
    }


    Serial.println("SIZE: " + size);
    Serial.println("\nIN PACKET LOOP: ");
    for (int i = 0; i < 154; i++) {
        if(i%10 == 0 ) Serial.println();
        Serial.print(packet[i]);
        Serial.print(":");
    }
    Serial.println("\n----------------------------------------\n");

    return packet;

}