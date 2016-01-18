#include <Arduino.h>
#include "Frame.h"
Frame::Frame(const byte *_destAddr, const byte *_srcAddr, const byte *_srcV6,
             const byte *_destV6, const byte  *_sendPort, const byte *_receivePort) {
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

const byte* Frame::getSrcV6() {
    return srcV6;
}

const byte* Frame::getDestV6() {
    return destV6;
}

const byte * Frame::getSendPort() {
    return sendPort;
}

byte * Frame::getReceivePort() {
    return receivePort;
}

uint16_t Frame::getSize() {
    return size;
}

byte * Frame::getTCPPacket(const byte* data, const bool ifSyn, const bool ifAck, const bool ifRes,
                           const bool ifFin, const uint16_t sizeOfData, const byte* destPort) {

    size = 74 + sizeOfData;
    byte* packet = new byte[size];

    static byte frameTemplate[]{
            //MAC addresses
            0x86, 0xdd,
            0x60,
            0x00,
            0x00,
            0x01,
            0x00,
            0x14,
            0x06,
            0x40,
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
    //IPv6 Source Address
    for (int k = 0; k < 16; k++) {
        packet[k+22] = srcV6[k];
    }
    //IPv6 Receive Address
    for (int k = 0; k < 16; k++) {
        packet[k+38] = destV6[k];
    }
    //source port
    receivePort[0] = destPort[0];
    receivePort[1] = destPort[1];
    packet[54] = sendPort[0];
    packet[55] = sendPort[1];
    packet[56] = receivePort[0];
    packet[57] = receivePort[1];

    delete []destPort;

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

    return packet;

}