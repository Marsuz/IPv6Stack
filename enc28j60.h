// Microchip ENC28J60 Ethernet Interface Driver
// Author: Pascal Stang
// Modified by: Guido Socher
// Copyright: GPL V2
//
// This driver provides initialization and transmit/receive
// functions for the Microchip ENC28J60 10Mb Ethernet Controller and PHY.
// This chip is novel in that it is a full MAC+PHY interface all in a 28-pin
// chip, using an SPI interface to the host processor.
//
// 2010-05-20 <jc@wippler.nl>
/** @file */

#ifndef ENC28J60_H
#define ENC28J60_H

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

/** This class provide low-level interfacing with the ENC28J60 network interface. This is used by the EtherCard class and not intended for use by (normal) end users. */
class ENC28J60 {
public:
    static uint8_t buffer[]; //!< Data buffer (shared by recieve and transmit)
    static uint16_t bufferSize; //!< Size of data buffer
    static bool broadcast_enabled; //!< True if broadcasts enabled (used to allow temporary disable of broadcast for DHCP or other internal functions)
    static bool promiscuous_enabled; //!< True if promiscuous mode enabled (used to allow temporary disable of promiscuous mode)
//    static byte frameToSend[];


    static uint8_t* tcpOffset () { return buffer + 0x36; } //!< Pointer to the start of TCP payload

    /**   @brief  Initialise SPI interface
    *     @note   Configures Arduino pins as input / output, etc.
    */
    static void initSPI ();

    /**   @brief  Initialise network interface
    *     @param  size Size of data buffer
    *     @param  macaddr Pointer to 6 byte hardware (MAC) address
    *     @param  csPin Arduino pin used for chip select (enable network interface SPI bus). Default = 8
    *     @return <i>uint8_t</i> ENC28J60 firmware version or zero on failure.
    */
    static uint8_t customInitialize(const uint16_t size, const uint8_t* macaddr);


    /**   @brief  Sends data to network interface
    *     @param  len Size of data to send
    *     @note   Data buffer is shared by recieve and transmit functions
    */
    static byte* customSend ();

    /**   @brief  Copy recieved packets to data buffer
    *     @return <i>uint16_t</i> Size of recieved data
    *     @note   Data buffer is shared by recieve and transmit functions
    */

    static uint16_t customReceive();

    static uint32_t sendTCPSyn();

    static uint32_t receiveTCPSynAck();

    static uint32_t sendTCPAck(uint32_t seqNum);

    static uint16_t calc_checksum(const byte* gPB, uint8_t off, uint16_t len);

    static uint32_t calc_seqnum(const byte* gPB, int lbound, int ubound);

    static void readPacket(uint16_t len);


    static void print_source_mac(byte* packet);

    static void print_dest_mac(byte* packet);

    static void print_source_ip(byte*packet);

    static void createFrame(const byte *srcAddr, const byte *destAddr, const byte *srcV6, const byte *destV6, const byte  *sendPort, const byte *receivePort);
};

typedef ENC28J60 Ethernet; //!< Define alias Ethernet for ENC28J60


/** Workaround for Errata 13.
*   The transmission hardware may drop some packets because it thinks a late collision
*   occurred (which should never happen if all cable length etc. are ok). If setting
*   this to 1 these packages will be retried a fixed number of times. Costs about 150bytes
*   of flash.
*/
#define ETHERCARD_RETRY_LATECOLLISIONS 0

/** Enable pipelining of packet transmissions.
*   If enabled the packetSend function will not block/wait until the packet is actually
*   transmitted; but instead this wait is shifted to the next time that packetSend is
*   called. This gives higher performance; however in combination with 
*   ETHERCARD_RETRY_LATECOLLISIONS this may lead to problems because a packet whose
*   transmission fails because the ENC-chip thinks that it is a late collision will
*   not be retried until the next call to packetSend.
*/
#define ETHERCARD_SEND_PIPELINING 0
#endif
