#ifndef ENC28J60_H
#define ENC28J60_H

#include "Frame.h"

// buffer boundaries applied to internal 8K ram
// the entire available packet buffer space is allocated

#define RXSTART_INIT        0x0000  // start of RX buffer, (must be zero, Rev. B4 Errata point 5)
#define RXSTOP_INIT         0x0BFF  // end of RX buffer, room for 2 packets
 
#define TXSTART_INIT        0x0C00  // start of TX buffer, room for 1 packet
#define TXSTOP_INIT         0x11FF  // end of TX buffer

#define SCRATCH_START       0x1200  // start of scratch area
#define SCRATCH_LIMIT       0x2000  // past end of area, i.e. 3 Kb
#define SCRATCH_PAGE_SHIFT  6       // addressing is in pages of 64 bytes
#define SCRATCH_PAGE_SIZE   (1 << SCRATCH_PAGE_SHIFT)
#define SCRATCH_PAGE_NUM    ((SCRATCH_LIMIT-SCRATCH_START) >> SCRATCH_PAGE_SHIFT)
#define SCRATCH_MAP_SIZE    (((SCRATCH_PAGE_NUM % 8) == 0) ? (SCRATCH_PAGE_NUM / 8) : (SCRATCH_PAGE_NUM/8+1))

// area in the enc memory that can be used via enc_malloc; by default 0 bytes; decrease SCRATCH_LIMIT in order
// to use this functionality
#define ENC_HEAP_START      SCRATCH_LIMIT
#define ENC_HEAP_END        0x2000
#define TCP_SOURCE_PORT1 54
#define TCP_SOURCE_PORT2 55

class ENC28J60 {
public:
    static uint8_t buffer[]; //!< Data buffer (shared by recieve and transmit)
    static uint16_t bufferSize; //!< Size of data buffer

    static void initSPI (); //Initialize SPI and configure Arduino pins

    static uint8_t initialize(const uint16_t size, const uint8_t* macaddr); //Initialize network interface

    static void send (byte* frameToSend, uint16_t size); //Sends data to network interface

//    static void sendTestFrame();

    static uint16_t receive(); //Copy received packets to data buffer

    static void seq_plus_payload_to_ack(uint32_t payload, byte *frameToSend);

    static void process_tcp_request(uint32_t pos);

    static void send_tcp_ack();

    static uint32_t packetLoop(uint16_t plen);


    static uint16_t calc_checksum(byte* gPB, uint8_t off, uint16_t len);

    static void add_to_seqnum(uint32_t number, byte *frameToSend);

    static uint32_t get_seqnum(int lbound, int ubound);

    static void readPacket(uint16_t len);


    static void print_source_mac(byte* packet);

    static void print_dest_mac(byte* packet);

    static void print_source_ip(byte*packet);

    static void createFrame(const byte *srcAddr, const byte *destAddr, const byte *srcV6,
                            const byte *destV6, const byte  *sendPort, const byte *receivePort);

    static void printTempHum();

    static byte* getPort();
};

typedef ENC28J60 Ethernet; //!< Define alias Ethernet for ENC28J60

#endif
