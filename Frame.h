//
// Created by Marcin on 2016-01-16.
//

#ifndef IPV6STACK_FRAME_H
#define IPV6STACK_FRAME_H

#endif //IPV6STACK_FRAME_H


class Frame {

    private:
    byte *srcAddr;
    byte *destAddr;
    byte *srcV6;
    byte *destV6;
    byte *sendPort;
    byte *receivePort;

    public:

    void Frame(byte *, byte *, byte *, byte *, byte *, byte *);
    void ~Frame();
    byte* getSrcAddr();
    byte* getDestAddr();
    byte* getSrcV6();
    byte* getDestV6();
    byte* getSendPort();
    byte* getReceivePort();

    byte* setSrcAddr(byte*);
    byte* setDestAddr(byte*);
    byte* setSrcV6(byte*);
    byte* setDestV6(byte*);
    byte* setSendPort(byte*);
    byte* setReceivePort(byte*);

    byte* getTCPPacket(byte*, bool, bool, bool);


};


