#ifndef IPV6STACK_FRAME_H
#define IPV6STACK_FRAME_H

#endif //IPV6STACK_FRAME_H

#include <Arduino.h>


class Frame {

    private:
    byte *srcAddr;
    byte *destAddr;
    byte *srcV6;
    byte *destV6;
    byte *sendPort;
    byte *receivePort;
    uint16_t size;
    public:

    Frame( byte *_destAddr,  byte *_srcAddr,  byte *_srcV6,
           byte *_destV6,  byte  *_sendPort, byte *_receivePort);
    ~Frame();
    byte* getSrcAddr();
    byte* getDestAddr();
    byte* getSrcV6();
    byte* getDestV6();
    byte* getSendPort();
    byte* getReceivePort();
    void setSrcAddr(byte * addr);
    void setDestAddr(byte * addr);
    void setSrcV6(byte * addr);
    void setDestV6(byte * addr);
    void setSendPort(byte * port);
    void setReceivePort(byte * port);
    uint16_t getSize();

    byte* getTCPPacket( byte* data,   bool ifSyn,   bool ifAck,
                         bool ifRes,  bool ifFinm,  uint16_t sizeOfData);


};


