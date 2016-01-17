//
// Created by Marcin on 2016-01-16.
//

#include "Frame.h"


Frame::Frame(byte *_srcAddr, byte *_destAddr, byte *_srcV6,
             byte *_destV6, byte  *_sendPort, byte *_receivePort) : {
    srcAddr = _srcAddr;
    destAddr = _destAddr;
    srcV6 = _srcV6;
    destV6 = _destV6;
    sendPort = _sendPort;
    receivePort = _receivePort;
}

::Frame::byte * Frame::getSrcAddr() {
    return srcAddr;
}

::Frame::byte * Frame::getDestAddr() {
    return destAddr;
}

::Frame::byte* Frame::getSrcV6() {
    return srcV6;
}

::Frame::byte* Frame::getDestV6() {
    return destV6;
}

::Frame::byte * Frame::getSendPort() {
    return sendPort;
}

::Frame::byte * Frame::getReceivePort() {
    return receivePort;
}

::Frame::byte * Frame::getTCPPacket(byte* data, bool ifSyn, bool ifAck, bool ifRes) {

    int size = (sizeof(data)/sizeof(*data));
    byte packet = new (nothrow) byte[74 + size];
    static byte frameTemplate[]{
            //MAC addresses
            0x86, 0xdd,
            0x60,
            0x00,
            0x00,
            0x01,
            0x00,
            0x64,
            0x06,
            0x40,
            //IPv6 addresses
            //ports
            0x00, 0x00, 0x00, 0x02,//Sequence number, theoretically random
            0x00, 0x00, 0x00, 0x00,//Acknowledgment, doesn't matter with SYN (right?)
            0b01010000,
            0b00000010,
            0x00, 0xFF,
            0x00, 0x00, //checksum
            0x00, 0x00 //URG not set, so value here doesn't matter
    };

    for (int k = 0; k < 6; k++) {
        packet[k] = srcAddr[k];
    }
    //destination address
    for (int k = 0; k < 6; k++) {
        packet[k+6] = destAddr[k];
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
        packet[64] = packet[64] | (0b00000010);
    }
    if(ifAck) {
        packet[64] = packet[64] | (0b00010000);
    }
    if(ifRes) {
        packet[64] = 0b00000100;
    }

    return packet;

}