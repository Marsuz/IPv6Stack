//
// Created by Marcin on 2016-01-16.
//

#ifndef IPV6STACK_FRAME_H
#define IPV6STACK_FRAME_H

#endif //IPV6STACK_FRAME_H

#include <Arduino.h>


class Frame {

    private:
    const byte *srcAddr;
    const byte *destAddr;
    const byte *srcV6;
    const byte *destV6;
    const byte *sendPort;
    const byte *receivePort;

    public:

    Frame(const byte *_srcAddr, const byte *_destAddr, const byte *_srcV6,
          const byte *_destV6, const byte  *_sendPort, const byte *_receivePort);
    ~Frame();
    const byte* getSrcAddr();
    const byte* getDestAddr();
    const byte* getSrcV6();
    const byte* getDestV6();
    const byte* getSendPort();
    const byte* getReceivePort();

//    byte* setSrcAddr(byte*);
//    byte* setDestAddr(byte*);
//    byte* setSrcV6(byte*);
//    byte* setDestV6(byte*);
//    byte* setSendPort(byte*);
//    byte* setReceivePort(byte*);

    byte* getTCPPacket(const byte* data, const  bool ifSyn, const  bool ifAck, const  bool ifRes);


};


