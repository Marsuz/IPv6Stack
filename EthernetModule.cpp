#include "enc28j60.h"

#if ARDUINO >= 100
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
#include "enc28j60.h"

void initialization(){

    static byte myip[] = { 0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4, 0x54, 0xBE };
// gateway ip address
    static byte gwip[] = { 0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4, 0x54, 0xBD };
#endif

// ethernet mac address - must be unique on your network
    static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };

    static byte ipv6_header[] = { 0x60, // Version 4 bits, Traffic Class 8 bits
                                  0x00, // Flow Label 20 bits
                                  0x00,
                                  0x01,
                                  0x00, // Payload Length 16 bits
                                  0x14,
                                  0x00, // Next Header
                                  0x00, // Hop Limit
                                  0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4, 0x54, 0xBE, // Source Address
                                  0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4, 0x54, 0xBD // Destination Address
    };

    //Initialization
    //MAC Initialization
    writeRegByte(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS); //enabling receiving frames and flow control
    writeRegByte(MACON2, 0x00);
    writeOp(ENC28J60_BIT_FIELD_SET, MACON3,
            MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN); //automatic padding, crc, frame length status reporting
    writeReg(MAMXFL, 1500); //Max frame length

    writeReg(MAIPG, 0x0C12); //half-duplex
    writeRegByte(MABBIPG, 0x12); //half-duplex

    static byte macaddr[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };

    writeRegByte(MAADR5, macaddr[0]); //setting MAC address
    writeRegByte(MAADR4, macaddr[1]);
    writeRegByte(MAADR3, macaddr[2]);
    writeRegByte(MAADR2, macaddr[3]);
    writeRegByte(MAADR1, macaddr[4]);
    writeRegByte(MAADR0, macaddr[5]);

    //PHY Initialization
    writePhy(PHCON2, PHCON2_HDLDIS); //Half-Duplex Automatic Loopback Disable

    writeReg(ETXST, TXSTART_INIT); //Transmit buffer start
    writeReg(ETXND, TXSTOP_INIT); //Transmit buffer end
    writeReg(ERXST, RXSTART_INIT); //Receive buffer start
    writeReg(ERXRDPT, RXSTART_INIT); //Buffer read pointer
    writeReg(ERXND, RXSTOP_INIT); //receive buffer End
}

void sendPackage(){
    uint16_t len;

//TODO Buffer needed

    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST); //Transmit Logic is held in Reset
    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST); //normal operation
    writeOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF|EIR_TXIF); //Interrupt - No transmit error nor transmit interrupt pending

    writeReg(EWRPT, TXSTART_INIT);
    writeReg(ETXND, TXSTART_INIT+len);
    writeOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
    writeBuf(len, buffer);

    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS); //Transmit Request to Send

}
