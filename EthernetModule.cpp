#include "enc28j60.h"   
#include <Arduino.h> // Arduino 1.0

void ENC28J60::customInitialize(uint16_t size, const uint8_t *macaddr) {
//
//    //bufferSize = size;
//    uint8_t csPin = 8;
//    if (bitRead(SPCR, SPE) == 0)
//        initSPI();
//    selectPin = csPin;
//    pinMode(selectPin, OUTPUT);
//    disableChip();
//
//    writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
//    delay(2); // errata B7/2
//    while (!readOp(ENC28J60_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY);
//
//    //Initialization
//    //MAC Initialization
//    writeRegByte(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS); //enabling receiving frames and flow control
//    writeRegByte(MACON2, 0x00);
//    writeOp(ENC28J60_BIT_FIELD_SET, MACON3,
//            MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN); //automatic padding, crc, frame length status reporting
//    writeReg(MAMXFL, 1500); //Max frame length
//
//    writeReg(MAIPG, 0x0C12); //half-duplex
//    writeRegByte(MABBIPG, 0x12); //half-duplex
//
//    static byte macaddr[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };
//
//    writeRegByte(MAADR5, macaddr[0]); //setting MAC address
//    writeRegByte(MAADR4, macaddr[1]);
//    writeRegByte(MAADR3, macaddr[2]);
//    writeRegByte(MAADR2, macaddr[3]);
//    writeRegByte(MAADR1, macaddr[4]);
//    writeRegByte(MAADR0, macaddr[5]);
//
//    //PHY Initialization
//    writePhy(PHCON2, PHCON2_HDLDIS); //Half-Duplex Automatic Loopback Disable
//
//    SetBank(ECON1); //?
//    writeOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
//    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
//
//    writeReg(ETXST, TXSTART_INIT); //Transmit buffer start
//    writeReg(ETXND, TXSTOP_INIT); //Transmit buffer end
//    writeReg(ERXST, RXSTART_INIT); //Receive buffer start
//    writeReg(ERXRDPT, RXSTART_INIT); //Buffer read pointer
//    writeReg(ERXND, RXSTOP_INIT); //receive buffer End
//    return 1;

    bufferSize = size;
    if (bitRead(SPCR, SPE) == 0)
        initSPI();
    selectPin = csPin;
    pinMode(selectPin, OUTPUT);
    disableChip();

    writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
    delay(2); // errata B7/2
    while (!readOp(ENC28J60_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY);

    writeReg(ERXST, RXSTART_INIT);
    writeReg(ERXRDPT, RXSTART_INIT);
    writeReg(ERXND, RXSTOP_INIT);
    writeReg(ETXST, TXSTART_INIT);
    writeReg(ETXND, TXSTOP_INIT);

    writeRegByte(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_PMEN | ERXFCON_BCEN);
    writeReg(EPMM0, 0x303f);
    writeReg(EPMCS, 0xf7f9);
    writeRegByte(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
    writeRegByte(MACON2, 0x00);
    writeOp(ENC28J60_BIT_FIELD_SET, MACON3,
            MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);
    writeReg(MAIPG, 0x0C12);
    writeRegByte(MABBIPG, 0x12);
    writeReg(MAMXFL, MAX_FRAMELEN);
    writeRegByte(MAADR5, macaddr[0]);
    writeRegByte(MAADR4, macaddr[1]);
    writeRegByte(MAADR3, macaddr[2]);
    writeRegByte(MAADR2, macaddr[3]);
    writeRegByte(MAADR1, macaddr[4]);
    writeRegByte(MAADR0, macaddr[5]);
    writePhy(PHCON2, PHCON2_HDLDIS);
    SetBank(ECON1);
    writeOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE | EIE_PKTIE);
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

    byte rev = readRegByte(EREVID);
    // microchip forgot to step the number on the silcon when they
    // released the revision B7. 6 is now rev B7. We still have
    // to see what they do when they release B8. At the moment
    // there is no B8 out yet
    if (rev > 5) ++rev;
    return rev;
}

void ENC28J60::customSend() {


    static byte myip[] = {0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4, 0x54,
                          0xBE};
// gateway ip address
    static byte gwip[] = {0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4, 0x54,
                          0xBD};
//#endif

// ethernet mac address - must be unique on your network
    static byte mymac[] = {0x74, 0x69, 0x69, 0x2D, 0x30, 0x31};
    static byte broadcastmac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    static byte type[] = {0x86, 0xdd};
    static byte ipv6_header[] = {0x60, // Version 4 bits, Traffic Class 8 bits
                                 0x00, // Flow Label 20 bits
                                 0x00,
                                 0x01,
                                 0x00, // Payload Length 16 bits
                                 0x14,
                                 0x00, // Next Header
                                 0x40, // Hop Limit
                                 0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4,
                                 0x54, 0xBE, // Source Address
                                 0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4,
                                 0x54, 0xBD // Destination Address
    };


    static byte tmp[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                         0x74, 0x69, 0x69, 0x2D, 0x30, 0x31,
                         0x86, 0xdd,
                         0x60,
                         0x00,
                         0x00,
                         0x01,
                         0x00,
                         0x00,
                         0x3b,
                         0x40,
                         0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4, 0x54, 0xBE,
                         0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xFC, 0x2A, 0x4A, 0x6D, 0xA4, 0x54,
                         0xBD};

    uint16_t len = 0x1b0;

    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST); //Transmit Logic is held in Reset
    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST); //normal operation
    writeOp(ENC28J60_BIT_FIELD_CLR, EIR,
            EIR_TXERIF | EIR_TXIF); //Interrupt - No transmit error nor transmit interrupt pending

    writeReg(EWRPT, TXSTART_INIT);
    writeReg(ETXND, TXSTART_INIT + (sizeof tmp));
    writeOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
    writeBuf(sizeof tmp, tmp);


    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS); //Transmit Request to Send
    return tmp;
}

void readPacket(uint16_t len) {
    byte *packet = new byte[len];
    readBuf(len, packet);
    print_dest_mac(packet);
    print_source_mac(packet);
    print_source_ip(packet);
}

void print_source_mac(byte* packet) {

    for(int i = 14; i < 20; i ++) {
        Serial.print(packet[i]);
    }
    Serial.println();

}

void print_dest_mac(byte* packet) {

    for(int i = 8; i < 14; i++) {
        Serial.print(packet[i]);
    }
    Serial.println();

}

void print_source_ip(byte*packet) {

    for(int i = 80; i < 96; i++) {
        Serial.print(packet[i]);
        if(i > 80 && i%2 == 0){
            Serial.print(":");
        }
    }
    Serial.println();
}


