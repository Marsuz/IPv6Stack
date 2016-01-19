#include <Arduino.h> // Arduino 1.0
#include "enc28j60.h"
#include "Adafruit_HDC1000.h"
#include <Wire.h>

uint16_t ENC28J60::bufferSize;

static uint32_t seq = 1;
static Frame* packet;

// ENC28J60 Control Registers
// Control register definitions are a combination of address,
// bank number, and Ethernet/MAC/PHY indicator bits.
// - Register address        (bits 0-4)
// - Bank number        (bits 5-6)
// - MAC/PHY indicator        (bit 7)
#define ADDR_MASK        0x1F
#define BANK_MASK        0x60
// All-bank registers
#define EIE              0x1B
#define EIR              0x1C
#define ESTAT            0x1D
#define ECON2            0x1E
#define ECON1            0x1F
// Bank 0 registers
#define ERDPT           (0x00|0x00)
#define EWRPT           (0x02|0x00)
#define ETXST           (0x04|0x00)
#define ETXND           (0x06|0x00)
#define ERXST           (0x08|0x00)
#define ERXND           (0x0A|0x00)
#define ERXRDPT         (0x0C|0x00)
// Bank 1 registers
#define EPMM0            (0x08|0x20)
#define EPMCS           (0x10|0x20)
#define ERXFCON          (0x18|0x20)
#define EPKTCNT          (0x19|0x20)
// Bank 2 registers
#define MACON1           (0x00|0x40|0x80)
#define MACON2           (0x01|0x40|0x80)
#define MACON3           (0x02|0x40|0x80)
#define MABBIPG          (0x04|0x40|0x80)
#define MAIPG           (0x06|0x40|0x80)
#define MAMXFL          (0x0A|0x40|0x80)
#define MICON            (0x11|0x40|0x80)
#define MICMD            (0x12|0x40|0x80)
#define MIREGADR         (0x14|0x40|0x80)
#define MIWR            (0x16|0x40|0x80)
#define MIRD            (0x18|0x40|0x80)
// Bank 3 registers
#define MAADR1           (0x00|0x60|0x80)
#define MAADR0           (0x01|0x60|0x80)
#define MAADR3           (0x02|0x60|0x80)
#define MAADR2           (0x03|0x60|0x80)
#define MAADR5           (0x04|0x60|0x80)
#define MAADR4           (0x05|0x60|0x80)
#define EBSTCON          (0x07|0x60)
#define MISTAT           (0x0A|0x60|0x80)
#define EREVID           (0x12|0x60)

// ENC28J60 ERXFCON Register Bit Definitions
#define ERXFCON_UCEN     0x80
#define ERXFCON_ANDOR    0x40
#define ERXFCON_CRCEN    0x20
#define ERXFCON_PMEN     0x10
#define ERXFCON_BCEN     0x01
#define EIE_INTIE        0x80
#define EIE_PKTIE        0x40
// ENC28J60 EIR Register Bit Definitions
#define EIR_TXIF         0x08
#define EIR_TXERIF       0x02
// ENC28J60 ESTAT Register Bit Definitions
#define ESTAT_CLKRDY     0x01
// ENC28J60 ECON2 Register Bit Definitions
#define ECON2_PKTDEC     0x40
// ENC28J60 ECON1 Register Bit Definitions
#define ECON1_TXRST      0x80
#define ECON1_TXRTS      0x08
#define ECON1_RXEN       0x04
#define ECON1_BSEL1      0x02
#define ECON1_BSEL0      0x01
// ENC28J60 MACON1 Register Bit Definitions
#define MACON1_TXPAUS    0x08
#define MACON1_RXPAUS    0x04
#define MACON1_MARXEN    0x01
// ENC28J60 MACON3 Register Bit Definitions
#define MACON3_PADCFG0   0x20
#define MACON3_TXCRCEN   0x10
#define MACON3_FRMLNEN   0x02
// ENC28J60 MICMD Register Bit Definitions
#define MICMD_MIIRD      0x01
// ENC28J60 MISTAT Register Bit Definitions
#define MISTAT_BUSY      0x01

// PHY registers
#define PHCON1           0x00
#define PHSTAT1          0x01
#define PHCON2           0x10

// ENC28J60 PHY PHCON2 Register Bit Definitions
#define PHCON2_HDLDIS    0x0100

// SPI operation codes
#define ENC28J60_READ_CTRL_REG       0x00
#define ENC28J60_READ_BUF_MEM        0x3A
#define ENC28J60_WRITE_CTRL_REG      0x40
#define ENC28J60_WRITE_BUF_MEM       0x7A
#define ENC28J60_BIT_FIELD_SET       0x80
#define ENC28J60_BIT_FIELD_CLR       0xA0
#define ENC28J60_SOFT_RESET          0xFF

// max frame length which the controller will accept:
// (note: maximum ethernet frame length would be 1518)

#define MAX_FRAMELEN      1500

#define IP_PAYLOAD_LEN_H 0x12
#define IP_PAYLOAD_LEN_L 0x13

#define TCP_FLAGS_FIN_V 0b00000001
#define TCP_FLAGS_SYN_V 0b00000010
#define TCP_FLAGS_ACK_ONLY 0b00010000
#define TCP_FLAGS_SYN_ACK 0b00010010
#define TCP_FLAGS_ACK_FIN 0b00010001
#define TCP_FLAGS_P 0x43

#define TCP_HEADER_LEN_P 0x3c
#define TCP_SRC_PORT_H_P 0x38
#define TCP_SEQ_P 0x3a
#define TCP_ACK_P 0x3e

static Adafruit_HDC1000 hdc = Adafruit_HDC1000();
static byte Enc28j60Bank;
static byte selectPin;
static uint8_t tcp_state;

//SPI initialize
void ENC28J60::initSPI() {
    //setting SPI communication with peripheral devices
    pinMode(SS, OUTPUT); //Slave Select
    digitalWrite(SS, HIGH); //Sending HIGH to SS
    pinMode(MOSI, OUTPUT); //The Master line for sending data to the peripherals
    pinMode(SCK, OUTPUT); //Serial Clock
    pinMode(MISO, INPUT); //The slave line for sending data to the Master

    digitalWrite(MOSI, HIGH);
    digitalWrite(MOSI, LOW);
    digitalWrite(SCK, LOW);

    SPCR = bit(SPE) | bit(MSTR); // SPI Controll Register, SPE Enables SPI when 1, Arduino is set in Master Mode  /8 MHz @ 16
    bitSet(SPSR, SPI2X); //SPI Status Register, Doubles the speed of SPI
}

static void enableChip() {
    cli(); //disable global interrupts
    digitalWrite(selectPin, LOW);
}

static void disableChip() {
    digitalWrite(selectPin, HIGH);
    sei(); //enable global interrupts
}

static void xferSPI(byte data) { //SPI Transaction
    SPDR = data; //Starting the transmission, SPI Data Register for sending to MOSI and receiving from MISO,
    while (!(SPSR & (1 << SPIF))); //Wait for the end of transmission
}

static byte readOp(byte op, byte address) {
    enableChip();
    xferSPI(op | (address & ADDR_MASK));
    xferSPI(0x00);
    if (address & 0x80)
        xferSPI(0x00);
    byte result = SPDR;
    disableChip();
    return result;
}

static void writeOp(byte op, byte address, byte data) {
    enableChip();
    xferSPI(op | (address & ADDR_MASK));
    xferSPI(data);
    disableChip();
}

static void readBuf(uint16_t len, byte *data) {
    uint8_t nextbyte;

    enableChip();
    if (len != 0) {
        xferSPI(ENC28J60_READ_BUF_MEM);

        SPDR = 0x00;
        while (--len) {
            while (!(SPSR & (1 << SPIF)));
            nextbyte = SPDR;
            SPDR = 0x00;
            *data++ = nextbyte;
        }
        while (!(SPSR & (1 << SPIF)));
        *data++ = SPDR;
    }
    disableChip();
}

static void writeBuf(uint16_t len, const byte *data) {
    enableChip();
    if (len != 0) {
        xferSPI(ENC28J60_WRITE_BUF_MEM);

        SPDR = *data++;
        while (--len) {
            uint8_t nextbyte = *data++;
            while (!(SPSR & (1 << SPIF)));
            SPDR = nextbyte;
        };
        while (!(SPSR & (1 << SPIF)));
    }
    disableChip();
}

static void SetBank(byte address) {
    if ((address & BANK_MASK) != Enc28j60Bank) {
        writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_BSEL1 | ECON1_BSEL0);
        Enc28j60Bank = address & BANK_MASK;
        writeOp(ENC28J60_BIT_FIELD_SET, ECON1, Enc28j60Bank >> 5);
    }
}

static byte readRegByte(byte address) {
    SetBank(address);
    return readOp(ENC28J60_READ_CTRL_REG, address);
}

static uint16_t readReg(byte address) {
    return readRegByte(address) + (readRegByte(address + 1) << 8);
}

static void writeRegByte(byte address, byte data) {
    SetBank(address);
    writeOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

static void writeReg(byte address, uint16_t data) {
    writeRegByte(address, data);
    writeRegByte(address + 1, data >> 8);
}

static uint16_t readPhyByte(byte address) {
    writeRegByte(MIREGADR, address);
    writeRegByte(MICMD, MICMD_MIIRD);
    while (readRegByte(MISTAT) & MISTAT_BUSY);
    writeRegByte(MICMD, 0x00);
    return readRegByte(MIRD + 1);
}

static void writePhy(byte address, uint16_t data) {
    writeRegByte(MIREGADR, address);
    writeReg(MIWR, data);
    while (readRegByte(MISTAT) & MISTAT_BUSY);
}


struct transmit_status_vector {
    uint8_t bytes[7];
};

uint8_t ENC28J60::initialize(uint16_t size, const uint8_t *macaddr) {

    tcp_state = 1;
    if (!hdc.begin()) {
        Serial.println("Couldn't find sensor!");
        while (1);
    }

    bufferSize = size;
    byte csPin = 8;
    if (bitRead(SPCR, SPE) == 0)
        initSPI();
    selectPin = csPin;
    pinMode(selectPin, OUTPUT);
    disableChip();

    writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
    delay(2); // errata B7/2
    while (!readOp(ENC28J60_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY);

    writeReg(ETXST, TXSTART_INIT); //Transmit buffer start
    writeReg(ETXND, TXSTOP_INIT); //Transmit buffer end
    writeReg(ERXST, RXSTART_INIT); //Receive buffer start
    writeReg(ERXRDPT, RXSTART_INIT); //Buffer read pointer
    writeReg(ERXND, RXSTOP_INIT); //receive buffer End

    //Initialization
    //MAC Initialization
    writeRegByte(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_PMEN | ERXFCON_BCEN);
    writeReg(EPMM0, 0x303f);
    writeReg(EPMCS, 0xf7f9);
    writeRegByte(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS); //enabling receiving frames and flow control
    writeRegByte(MACON2, 0x00);
    writeOp(ENC28J60_BIT_FIELD_SET, MACON3,
            MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN); //automatic padding, crc, frame length status reporting

    writeReg(MAIPG, 0x0C12); //half-duplex
    writeRegByte(MABBIPG, 0x12); //half-duplex

    writeReg(MAMXFL, MAX_FRAMELEN); //Max frame length
    writeRegByte(MAADR5, macaddr[0]); //setting MAC address
    writeRegByte(MAADR4, macaddr[1]);
    writeRegByte(MAADR3, macaddr[2]);
    writeRegByte(MAADR2, macaddr[3]);
    writeRegByte(MAADR1, macaddr[4]);
    writeRegByte(MAADR0, macaddr[5]);

    //PHY Initialization
    writePhy(PHCON2, PHCON2_HDLDIS); //Half-Duplex Automatic Loopback Disable

    SetBank(ECON1); //?
    writeOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE | EIE_PKTIE);
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

//    byte rev = readRegByte(EREVID);
//    // microchip forgot to step the number on the silcon when they
//    // released the revision B7. 6 is now rev B7. We still have
//    // to see what they do when they release B8. At the moment
//    // there is no B8 out yet
//    if (rev > 5) ++rev;
//    return rev;
    return 0;
}




void ENC28J60::send(byte *frameToSend, uint16_t size) {

        /*for (int k = 0; k < 80; k++) {
            frameToSend[k + 74] = data[k];
        }*/

        frameToSend[70] = 0;
        frameToSend[71] = 0;

        uint16_t checksum = calc_checksum(frameToSend, 22, size - 22);
        frameToSend[70] = checksum >> 8;
        frameToSend[71] = checksum;



        writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST); //Transmit Logic is held in Reset
        writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST); //normal operation
        writeOp(ENC28J60_BIT_FIELD_CLR, EIR,
                EIR_TXERIF | EIR_TXIF); //Interrupt - No transmit error nor transmit interrupt pending



        writeReg(EWRPT, TXSTART_INIT);
        writeReg(ETXND, TXSTART_INIT + size);
        writeOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
        writeBuf(size, frameToSend);


//    Serial.println("SIZE: " + size);
//    Serial.println("\nIN PACKET LOOP: ");
//    for (int i = 0; i < 80; i++) {
//        if(i%10 == 0 ) Serial.println();
//        Serial.print(frameToSend[i], HEX);
//        Serial.print(":");
//    }
//    Serial.println("\n----------------------------------------\n");

        writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS); //Transmit Request to Send
        delete [] frameToSend;
}

void ENC28J60::createFrame(const byte *srcAddr,const byte *destAddr,const byte *srcV6,
                           const byte *destV6, const byte  *sendPort, byte *receivePort) {

    packet = new Frame(srcAddr, destAddr, srcV6, destV6, sendPort, receivePort);

}

//void ENC28J60::sendTestFrame() {
//    Serial.print("Sending Test Frame\n");
//    byte data[2];
//    data[0] = 0x01;
//    data[1] = 0x02;
//    byte * frameToSend = packet->getTCPPacket(data, true, false, false, false, (uint16_t)2);
//    uint16_t size = packet->getSize();
//
//    send(frameToSend, size);
//}

uint16_t ENC28J60::receive() {
    static uint16_t gNextPacketPtr = RXSTART_INIT;
    uint16_t len = 0;
    if (readRegByte(EPKTCNT) > 0) {
//        Serial.print("received packet");
        writeReg(ERDPT, gNextPacketPtr);

        struct {
            uint16_t nextPacket;
            uint16_t byteCount;
            uint16_t status;
        } header;

        readBuf(sizeof header, (byte * ) & header);

        gNextPacketPtr = header.nextPacket;
        len = header.byteCount - 4; //remove the CRC count
        if (len > bufferSize - 1)
            len = bufferSize - 1;
        if ((header.status & 0x80) == 0)
            len = 0;
        else
            readBuf(len, buffer);
        buffer[len] = 0;

        writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
    }
    return len;
}

void ENC28J60::process_tcp_request(uint32_t pos) {

    send_tcp_ack();
    char* data = (char *) buffer + pos;
    if (strncmp("GET / ", data, 6) == 0) {// nothing specified
        http_post();
//        prepare_data();//TODO: prepare data to send via HTTP
        Serial.print("Processing tcp request\n");
    } else {
        http_post();
        Serial.print("Processing tcp request else\n");
    }

}

void ENC28J60::send_tcp_ack() {
    Serial.println("SENDING TCP ACK\n");
    byte data[0];
    packet->setReceivePort(buffer[TCP_SOURCE_PORT1], buffer[TCP_SOURCE_PORT2]);
    byte * frameToSend = packet->getTCPPacket(data, false, true, false, false, (uint16_t)0);
    uint16_t size = packet->getSize();
    uint32_t payload = get_payload();//TODO: 1. Get payload length of packet
    //uint32_t payload = 0;
    seq_plus_payload_to_ack(payload, frameToSend);
    add_to_seqnum((uint32_t)0, frameToSend);
    //here should be option to send packet without any data!
    send(frameToSend, size);
}

uint32_t ENC28J60::packetLoop(uint16_t plen) {

//    if (gPB[TCP_DST_PORT_H_P] == (port >> 8) && TODO: check if ports are consistent
//        gPB[TCP_DST_PORT_L_P] == ((uint8_t) port))
//    {   //Packet targetted at specified port

//    Serial.println("\nIN PACKET LOOP: ");
//    for (int i = 0; i < 154; i++) {
//        if(i%10 == 0 ) Serial.println();
//        Serial.print(buffer[i], HEX);
//        Serial.print(":");
//    }
//    Serial.println("\n----------------------------------------\n");

    switch (tcp_state) {

        case 1 :
            if(buffer[TCP_FLAGS_P] & TCP_FLAGS_SYN_V) { //send SYN+ACK
                tcp_state = 2;
                Serial.print("Packet Loop If\n");
                byte data[0];
                packet->setReceivePort(buffer[TCP_SOURCE_PORT1], buffer[TCP_SOURCE_PORT2]);
                byte * frameToSend = packet->getTCPPacket(data, true, true, false, false,(uint16_t) 0);


                uint16_t size = packet->getSize();
                seq_plus_payload_to_ack((uint32_t)1, frameToSend);
                //frameToSend[TCP_FLAGS_P] = TCP_FLAGS_ACK_ONLY;

                send(frameToSend, size);
            }
            break;

        case 2:
            if (buffer[TCP_FLAGS_P] & TCP_FLAGS_ACK_ONLY) {   //This is an acknowledgement to our SYN+ACK so let's start processing that payload
                Serial.println("Packet Loop else if\n");
                uint16_t info_data_len = 0;
                info_data_len = buffer[IP_PAYLOAD_LEN_H] << 8;
                info_data_len += buffer[IP_PAYLOAD_LEN_L];

                if (info_data_len > 0) {   //Got some data0
                    uint16_t pos = ((uint16_t) TCP_SRC_PORT_H_P +
                                    (buffer[TCP_HEADER_LEN_P] >> 4) * 4); // start of data field in TCP frame
                    if (pos <= plen) {
                        tcp_state = 3;
                        return pos;
                    }
                } /*else if (buffer[TCP_FLAGS_P] & TCP_FLAGS_ACK_FIN || buffer[TCP_FLAGS_P] & TCP_FLAGS_FIN_V) {
                    byte data[0];
                    packet->setReceivePort(buffer[TCP_SOURCE_PORT1], buffer[TCP_SOURCE_PORT2]);
                    byte *frameToSend = packet->getTCPPacket(data, false, true, false, true, (uint16_t) 0);
                    uint16_t size = packet->getSize();
                    seq_plus_payload_to_ack((uint32_t) 1, frameToSend);
                    send(frameToSend, size);
                    tcp_state = 1;
                }*/
            }
            break;

        case 3:
            tcp_state = 4;
            byte data[0];
            byte *frameToSend = packet->getTCPPacket(data, false, false , false, true, 0); // sending fin packet
            uint16_t size = packet->getSize();
            seq_plus_payload_to_ack((uint32_t) 1, frameToSend);
            send(frameToSend, size);
            break;

        case 4:

            if(buffer[TCP_FLAGS_P] & TCP_FLAGS_ACK_ONLY) {
                tcp_state = 5;
            }
            break;

        case 5:

            if(buffer[TCP_FLAGS_P] & TCP_FLAGS_FIN_V) {
                tcp_state = 1;
                byte data[0];
                packet->setReceivePort(buffer[TCP_SOURCE_PORT1], buffer[TCP_SOURCE_PORT2]);
                byte *frameToSend = packet->getTCPPacket(data, false, true , false, false, 0); // sending ack packet after fin
                uint16_t size = packet->getSize();
                seq_plus_payload_to_ack((uint32_t) 1, frameToSend);
                send(frameToSend, size);
            }
            break;

        default: {}

    }
    return 0;

}

void ENC28J60::seq_plus_payload_to_ack(uint32_t payload, byte *frameToSend) {
//    Serial.print("seq_plus_payload_to_ack\n");
    uint32_t seqNum = get_seqnum(58, 61);
    seqNum += payload;
//    Serial.println("\nACK\n");
//    Serial.println(seqNum);
    uint16_t div = 256;
    for(int i = 0; i < 4; i++) {
        frameToSend[TCP_ACK_P + (3 - i)] = seqNum % div;
        seqNum = seqNum >> 8;
    }

}


uint16_t ENC28J60::calc_checksum(byte *frameToSend, uint8_t off, uint16_t len) {
    const uint8_t *ptr = frameToSend + off;

    uint32_t sum = len - 26; //magic
    while (len > 1) {
        sum += (uint16_t)(((uint32_t) * ptr << 8) | *(ptr + 1));
        ptr += 2;
        len -= 2;
    }
    if (len)
        sum += ((uint32_t) * ptr) << 8;
    while (sum >> 16)
        sum = (uint16_t) sum + (sum >> 16);
    return ~(uint16_t) sum;
}

uint32_t ENC28J60::get_seqnum(int lbound, int ubound) {
//    Serial.print("get_sqgnum\n");
    uint32_t result = 0;
    for (int i = lbound; i <= ubound; i++) {
        result = result << 8;
        result += (uint32_t) buffer[i];
    }
    return result;
};

void ENC28J60::add_to_seqnum(uint32_t number, byte *frameToSend) {
    Serial.println("SEQ NUM: ");
    Serial.print(number);

    seq += number;
    uint16_t oct = 256;

    for(int i = 0; i < 4; i++) {
        frameToSend[TCP_SEQ_P + (3 - i)] = ((byte)((seq >> (8*i)) % oct));
    }

}

void ENC28J60::readPacket(uint16_t len) {
//    if(len == 0) return;
//    byte *packet = new byte[len];
//    readBuf(len, packet);
    Serial.print("\n");
    print_dest_mac(buffer);
    print_source_mac(buffer);
    print_source_ip(buffer);
}

void ENC28J60::print_source_mac(byte *packet) {

    for (int i = 0; i < 6; i++) {
        Serial.print(packet[i], HEX);
        if (i != 6) Serial.print(":");
    }
    Serial.println();

}

void ENC28J60::print_dest_mac(byte *packet) {

    for (int i = 6; i < 12; i++) {
        Serial.print(packet[i], HEX);
        if (i != 12) Serial.print(":");
    }
    Serial.println();

}

void ENC28J60::print_source_ip(byte *packet) {

    for (int i = 22; i < 38; i++) {
        Serial.print(packet[i], HEX);
        if (i > 22 && i % 2 == 1) {
            Serial.print(":");
        }
    }
    Serial.println();
}


void ENC28J60::printTempHum() {
    Serial.print("\nTemp: "); Serial.print(hdc.readTemperature());
    Serial.print("\t\tHum: "); Serial.println(hdc.readHumidity());

}


uint32_t ENC28J60::get_payload() {
    byte payload[2];
    payload[0] = buffer[18];
    payload[1] = buffer[19];

    uint32_t result = (uint32_t) (payload[0]);
    result = (result << 8);
    result += (uint32_t) (payload[1]);

    return result;
}

void ENC28J60::http_post() {
    Serial.println("\nSending http post\n");
//    byte data[] = "HTTP/1.1 200 OK\r\nServer: Apache/1.3.19 (Unix)\r\nAccept-Ranges: bytes\r\nContent-Length: 48\r\nKeep-Alive: timeout=15, max=100\r\nConnection: Keep-Alive\r\nContent-Type: text/html\r\n\r\n<html></html>\r\n\r\n";
    byte data[] = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<html></html>\r\n\r\n";
    packet->setReceivePort(buffer[TCP_SOURCE_PORT1], buffer[TCP_SOURCE_PORT2]);
    byte * frameToSend = packet->getTCPPacket(data, false, true, false, false, (uint16_t)(sizeof data));
    uint16_t size = packet->getSize();
    uint32_t payload = get_payload() ;
    seq_plus_payload_to_ack(payload,frameToSend);
    send(frameToSend, size);
    //add_to_seqnum((uint16_t)(sizeof data),frameToSend);
}



