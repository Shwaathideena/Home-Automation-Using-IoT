// ENC28J60 Driver
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "enc28j60.h"
//#include "wait.h"
//#include "ether.h"

// ------------------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------------------



//to know which current port we are using
uint16_t currTcpPortNum;

// ------------------------------------------------------------------------------
//  Structures
// ------------------------------------------------------------------------------

// This M4F is little endian
// Network byte order is big endian
// Must uint8_terpret uint16_ts in reverse order

struct enc28j60Frame // 4-bytes
{
  uint16_t size;
  uint16_t status;
  uint8_t data;
} *enc28j60;

struct etherFrame // 14-bytes
{
  uint8_t destAddress[6];
  uint8_t sourceAddress[6];
  uint16_t frameType;
  uint8_t data;
} *ether;

struct _ip // minimum 20 bytes
{
  uint8_t rev_size;
  uint8_t typeOfService;
  uint16_t length;
  uint16_t id;
  uint16_t flagsAndOffset;
  uint8_t ttl;
  uint8_t protocol;
  uint16_t headerChecksum;
  uint8_t sourceIp[4];
  uint8_t destIp[4];
} *ip;

struct _icmp
{
  uint8_t type;
  uint8_t code;
  uint16_t check;
  uint16_t id;
  uint16_t seq_no;
  uint8_t data;
} *icmp;

struct _arp
{
  uint16_t hardwareType;
  uint16_t protocolType;
  uint8_t hardwareSize;
  uint8_t protocolSize;
  uint16_t op;
  uint8_t sourceAddress[6];
  uint8_t sourceIp[4];
  uint8_t destAddress[6];
  uint8_t destIp[4];
} *arp;

struct _udp // 8 bytes
{
  uint16_t sourcePort;
  uint16_t destPort;
  uint16_t length;
  uint16_t check;
  uint8_t  data;
} *udp;

struct _tcp // 20 bytes plus Options or Data
{
  uint16_t sourcePort;
  uint16_t destPort;
  uint32_t seqNum;
  uint32_t ackNum;
  uint16_t hlengthf;
  uint16_t windowSize;
  uint16_t check;
  uint16_t UrgentPointer;
  uint32_t options1;
  uint32_t options2;
  uint32_t bal;
  uint8_t data[1530];
} *tcp;


uint8_t nextPacketLsb = 0x00;
uint8_t nextPacketMsb = 0x00;
uint8_t sequenceId;
uint32_t sum;
uint8_t macAddress[6] = {2,3,4,5,6,7};
uint8_t ipv4Address[4];
uint8_t parsedData[10];
uint8_t temp=0;
uint8_t hum=0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void waitMicrosecond(uint32_t ms)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

void spiWrite(uint8_t data)
{
    SSI2_DR_R = data;
    while (SSI2_SR_R & SSI_SR_BSY);
}

uint8_t spiRead()
{
    return SSI2_DR_R;
}

void etherCsOn()
{
    PIN_ETHER_CS = 0;
    __asm (" NOP");                    // allow line to settle
    __asm (" NOP");
    __asm (" NOP");
    __asm (" NOP");
}

void etherCsOff()
{
    PIN_ETHER_CS = 1;
}

void etherWriteReg(uint8_t reg, uint8_t data)
{
    etherCsOn();
    spiWrite(0x40 | (reg & 0x1F));
    spiRead();
    spiWrite(data);
    spiRead();
    etherCsOff();
}

uint8_t etherReadReg(uint8_t reg)
{
    uint8_t data;
    etherCsOn();
    spiWrite(0x00 | (reg & 0x1F));
    spiRead();
    spiWrite(0);
    data = spiRead();
    etherCsOff();
    return data;
}

void etherSetReg(uint8_t reg, uint8_t mask)
{
    etherCsOn();
    spiWrite(0x80 | (reg & 0x1F));
    spiRead();
    spiWrite(mask);
    spiRead();
    etherCsOff();
}

void etherClearReg(uint8_t reg, uint8_t mask)
{
    etherCsOn();
    spiWrite(0xA0 | (reg & 0x1F));
    spiRead();
    spiWrite(mask);
    spiRead();
    etherCsOff();
}

void etherSetBank(uint8_t reg)
{
    etherClearReg(ECON1, 0x03);
    etherSetReg(ECON1, reg >> 5);
}

void etherWritePhy(uint8_t reg, uint16_t data)
{
    etherSetBank(MIREGADR);
    etherWriteReg(MIREGADR, reg);
    etherWriteReg(MIWRL, data & 0xFF);
    etherWriteReg(MIWRH, (data >> 8) & 0xFF);
}

uint16_t etherReadPhy(uint8_t reg)
{
    uint16_t data, data2;
    etherSetBank(MIREGADR);
    etherWriteReg(MIREGADR, reg);
    etherWriteReg(MICMD, etherReadReg(MICMD) | MIIRD);
    waitMicrosecond(50);
    while ((etherReadReg(MISTAT) | MIBUSY) != 0);
    etherWriteReg(MICMD, etherReadReg(MICMD) & ~MIIRD);
    data = etherReadReg(MIRDL);
    data2 = etherReadReg(MIRDH);
    data |= (data2 << 8);
    return data;
}

void etherWriteMemStart()
{
    etherCsOn();
    spiWrite(0x7A);
    spiRead();
}

void etherWriteMem(uint8_t data)
{
    spiWrite(data);
    spiRead();
}

void etherWriteMemStop()
{
    etherCsOff();
}

void etherReadMemStart()
{
    etherCsOn();
    spiWrite(0x3A);
    spiRead();
}

uint8_t etherReadMem()
{
    spiWrite(0);
    return spiRead();
}

void etherReadMemStop()
{
    etherCsOff();
}

// Initializes ethernet device
// Uses order suggested in Chapter 6 of datasheet except 6.4 OST which is first here
void etherInit(uint8_t mode)
{
    // make sure that oscillator start-up timer has expired
    while ((etherReadReg(ESTAT) & CLKRDY) == 0) {}

    // disable transmission and reception of packets
    etherClearReg(ECON1, RXEN);
    etherClearReg(ECON1, TXRTS);

    // initialize receive buffer space
    etherSetBank(ERXSTL);
    etherWriteReg(ERXSTL, LOBYTE(0x0000));
    etherWriteReg(ERXSTH, HIBYTE(0x0000));
    etherWriteReg(ERXNDL, LOBYTE(0x1A09));
    etherWriteReg(ERXNDH, HIBYTE(0x1A09));

    // initialize receiver write and read ptrs
    // at startup, will write from 0 to 1A08 only and will not overwrite rd ptr
    etherWriteReg(ERXWRPTL, LOBYTE(0x0000));
    etherWriteReg(ERXWRPTH, HIBYTE(0x0000));
    etherWriteReg(ERXRDPTL, LOBYTE(0x1A09));
    etherWriteReg(ERXRDPTH, HIBYTE(0x1A09));
    etherWriteReg(ERDPTL, LOBYTE(0x0000));
    etherWriteReg(ERDPTH, HIBYTE(0x0000));

    // setup receive filter
    // always check CRC, use OR mode
    etherSetBank(ERXFCON);
    etherWriteReg(ERXFCON, (mode | 0x20) & 0xBF);
    // bring mac out of reset
    etherSetBank(MACON2);
    etherWriteReg(MACON2, 0);

    // enable mac rx, enable pause control for full duplex
    etherWriteReg(MACON1, TXPAUS | RXPAUS | MARXEN);

    // enable padding to 60 bytes (no runt packets)
    // add crc to tx packets, set full or half duplex
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWriteReg(MACON3, FULDPX | FRMLNEN | TXCRCEN | PAD60);
    else
        etherWriteReg(MACON3, FRMLNEN | TXCRCEN | PAD60);

    // leave MACON4 as reset

    // set maximum rx packet size
    etherWriteReg(MAMXFLL, LOBYTE(1518));
    etherWriteReg(MAMXFLH, HIBYTE(1518));

    // set back-to-back uint8_ter-packet gap to 9.6us
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWriteReg(MABBIPG, 0x15);
    else
        etherWriteReg(MABBIPG, 0x12);

    // set non-back-to-back uint8_ter-packet gap registers
    etherWriteReg(MAIPGL, 0x12);
    etherWriteReg(MAIPGH, 0x0C);

    // leave collision window MACLCON2 as reset

    // setup mac address
    etherSetBank(MAADR0);
    etherWriteReg(MAADR5, macAddress[0]);
    etherWriteReg(MAADR4, macAddress[1]);
    etherWriteReg(MAADR3, macAddress[2]);
    etherWriteReg(MAADR2, macAddress[3]);
    etherWriteReg(MAADR1, macAddress[4]);
    etherWriteReg(MAADR0, macAddress[5]);

    // initialize phy duplex
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWritePhy(PHCON1, PDPXMD);
    else
        etherWritePhy(PHCON1, 0);

    // disable phy loopback if in half-duplex mode
    etherWritePhy(PHCON2, HDLDIS);

    // set LEDA (link status) and LEDB (tx/rx activity)
    // stretch LED on to 40ms (default)
    etherWritePhy(PHLCON, 0x0472);

    // enable reception
    etherSetReg(ECON1, RXEN);
}

// Returns TRUE if packet received
uint8_t etherKbhit()
{
    return ((etherReadReg(EIR) & PKTIF) != 0);
}

// Returns up to max_size characters in data buffer
// Returns number of bytes copied to buffer
// Contents written are 16-bit size, 16-bit status, payload excl crc
uint16_t etherGetPacket(uint8_t data[], uint16_t max_size)
{
    uint16_t i = 0, size, tmp;

    // enable read from FIFO buffers
    etherReadMemStart();

    // get next pckt information
    nextPacketLsb = etherReadMem();
    nextPacketMsb = etherReadMem();

    // calc size
    // don't return crc, instead return size + status, so size is correct
    size = etherReadMem();
    data[i++] = size;
    tmp = etherReadMem();
    data[i++] = tmp;
    size |= (tmp << 8);

    // copy status + data
    if (size > max_size)
        size = max_size;
    while (i < size)
        data[i++] = etherReadMem();

    // end read from FIFO buffers
    etherReadMemStop();

    // advance read ptr
    etherSetBank(ERXRDPTL);
    etherWriteReg(ERXRDPTL, nextPacketLsb); // hw ptr
    etherWriteReg(ERXRDPTH, nextPacketMsb);
    etherWriteReg(ERDPTL, nextPacketLsb); // dma rd ptr
    etherWriteReg(ERDPTH, nextPacketMsb);

    // decrement packet counter so that PKTIF is maintained correctly
    etherSetReg(ECON2, PKTDEC);

    return size;
}

// Returns TRUE is rx buffer overflowed after correcting the problem
uint8_t etherIsOverflow()
{
    uint8_t err;
    err = (etherReadReg(EIR) & RXERIF) != 0;
    if (err)
        etherClearReg(EIR, RXERIF);
    return err;
}

// Writes a packet
bool etherPutPacket(uint8_t data[], uint16_t size)
{
    uint16_t i;

    // clear out any tx errors
    if ((etherReadReg(EIR) & TXERIF) != 0)
    {
        etherClearReg(EIR, TXERIF);
        etherSetReg(ECON1, TXRTS);
        etherClearReg(ECON1, TXRTS);
    }

    // set DMA start address
    etherSetBank(EWRPTL);
    etherWriteReg(EWRPTL, LOBYTE(0x1A0A));
    etherWriteReg(EWRPTH, HIBYTE(0x1A0A));

    // start FIFO buffer write
    etherWriteMemStart();

    // write control byte
    etherWriteMem(0);

    // write data
    for (i = 0; i < size; i++)
        etherWriteMem(data[i]);

    // stop write
    etherWriteMemStop();

    // request transmit
    etherWriteReg(ETXSTL, LOBYTE(0x1A0A));
    etherWriteReg(ETXSTH, HIBYTE(0x1A0A));
    etherWriteReg(ETXNDL, LOBYTE(0x1A0A+size));
    etherWriteReg(ETXNDH, HIBYTE(0x1A0A+size));
    etherClearReg(EIR, TXIF);
    etherSetReg(ECON1, TXRTS);

    // wait for completion
    while ((etherReadReg(ECON1) & TXRTS) != 0);

    // determine success
    return ((etherReadReg(ESTAT) & TXABORT) == 0);
}

// Calculate sum of words
// Must use getEtherChecksum to complete 1's compliment addition
void etherSumWords(void* data, uint16_t size_in_bytes)
{
    uint8_t* pData = (uint8_t*)data;
    uint16_t i;
    uint8_t phase = 0;
    uint16_t data_temp;
    for (i = 0; i < size_in_bytes; i++)
    {
        if (phase)
        {
            data_temp = *pData;
            sum += data_temp << 8;
        }
        else
          sum += *pData;
        phase = 1 - phase;
        pData++;
    }
}

// Completes 1's compliment addition by folding carries back uint8_to field
uint16_t getEtherChecksum()
{
    uint16_t result;
    // this is based on rfc1071
    while ((sum >> 16) > 0)
      sum = (sum & 0xFFFF) + (sum >> 16);
    result = sum & 0xFFFF;
    return ~result;
}

// Converts from host to network order and vice versa
uint16_t htons(uint16_t value)
{
    return ((value & 0xFF00) >> 8) + ((value & 0x00FF) << 8);
}

#define ntohs htons

// Determines whether packet is IP datagram
uint8_t etherIsIp(uint8_t data[])
{
    uint8_t ok;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    ok = (ether->frameType == 0x0008);
    if (ok)
    {
        sum = 0;
        etherSumWords(&ip->rev_size, (ip->rev_size & 0xF) * 4);
        ok = (getEtherChecksum() == 0);
    }
    return ok;
}

// Determines whether packet is unicast to this ip
// Must be an IP packet
bool etherIsIpUnicast(uint8_t data[])
{
    uint8_t i = 0;
    bool ok = true;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    while (ok && (i < 4))
    {
        ok = (ip->destIp[i] == ipv4Address[i]);
        i++;
    }
    return ok;
}

// Determines whether packet is ping request
// Must be an IP packet
uint8_t etherIsPingReq(uint8_t data[])
{
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    icmp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    return (ip->protocol == 0x01 && icmp->type == 8);
}

// Sends a ping response given the request data
void etherSendPingResp(uint8_t data[])
{
    uint8_t i, tmp;
    uint16_t icmp_size;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    icmp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    // swap source and destination fields
    for (i = 0; i < 6; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }
    for (i = 0; i < 4; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip ->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }
    // this is a response
    icmp->type = 0;
    // calc icmp checksum
    sum = 0;
    etherSumWords(&icmp->type, 2);
    icmp_size = ntohs(ip->length);
    icmp_size -= 24; // sub ip header and icmp code, type, and check
    etherSumWords(&icmp->id, icmp_size);
    icmp->check = getEtherChecksum();
    // send packet
    etherPutPacket((uint8_t*)ether, 14 + ntohs(ip->length));
}

// Determines whether packet is ARP
uint8_t etherIsArp(uint8_t data[])
{
    uint8_t ok;
    uint8_t i = 0;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    arp = (void*)&ether->data;
    ok = (ether->frameType == 0x0608);
    while (ok && (i < 4))
    {
        ok = (arp->destIp[i] == ipv4Address[i]);
        i++;
    }
    return ok;
}

// Sends an ARP response given the request data
void etherSendArpResp(uint8_t data[])
{
    uint8_t i, tmp;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    arp = (void*)&ether->data;
    // set op to response
    arp->op = 0x0200;
    // swap source and destination fields
    for (i = 0; i < 6; i++)
    {
        arp->destAddress[i] = arp->sourceAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = arp->sourceAddress[i] = macAddress[i];
    }
    for (i = 0; i < 4; i++)
    {
        tmp = arp->destIp[i];
        arp->destIp[i] = arp->sourceIp[i];
        arp->sourceIp[i] = tmp;
    }
    // send packet
    etherPutPacket((uint8_t*)ether, 42);
}

// Sends an ARP request
void etherSendArpReq(uint8_t data[], uint8_t ip[])
{
    uint8_t i;
    ether = (void*)data;
    arp = (void*)&ether->data;
    // fill ethernet frame
    for (i = 0; i < 6; i++)
    {
        ether->destAddress[i] = 0xFF;
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = 0x0608;
    // fill arp frame
    arp->hardwareType = 0x0100;
    arp->protocolType = 0x0008;
    arp->hardwareSize = 6;
    arp->protocolSize = 4;
    arp->op = 0x0100;
    for (i = 0; i < 6; i++)
    {
        arp->sourceAddress[i] = macAddress[i];
        arp->destAddress[i] = 0xFF;
    }
    for (i = 0; i < 4; i++)
    {
        arp->sourceIp[i] = ipv4Address[i];
        arp->destIp[i] = ip[i];
    }
    // send packet
    etherPutPacket(data, 42);
}

// Determines whether packet is UDP datagram
// Must be an IP packet
uint8_t etherIsUdp(uint8_t data[])
{
    uint8_t ok;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    udp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    ok = (ip->protocol == 0x11);
    if (ok)
    {
        // 32-bit sum over pseudo-header
        sum = 0;
        etherSumWords(ip->sourceIp, 8);
        tmp_int = ip->protocol;
        sum += (tmp_int & 0xff) << 8;
        etherSumWords(&udp->length, 2);
        // add udp header and data
        etherSumWords(udp, ntohs(udp->length));
        ok = (getEtherChecksum() == 0);
    }
    return ok;
}

// Gets pointer to UDP payload of frame
uint8_t* etherGetUdpData(uint8_t data[])
{
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    udp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    return &udp->data;
}

void etherCalcIpChecksum()
{
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();
}

// Send responses to a udp datagram
// destination port, ip, and hardware address are extracted from provided data
// uses destination port of received packet as destination of this packet
void etherSendUdpData(uint8_t data[], uint8_t* udp_data, uint8_t udp_size)
{
    uint8_t *copy_data;
    uint8_t i, tmp;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    udp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));
    // swap source and destination fields
    for (i = 0; i < 6; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }
    for (i = 0; i < 4; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }
    // set source port of resp will be dest port of req
    // dest port of resp will be left at source port of req
    // unusual nomenclature, but this allows a different tx
    // and rx port on other machine
    udp->sourcePort = udp->destPort;
    // adjust lengths
    ip->length = htons(((ip->rev_size & 0xF) * 4) + 8 + udp_size);
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();
    udp->length = htons(8 + udp_size);
    // copy data
    copy_data = &udp->data;
    for (i = 0; i < udp_size; i++)
        copy_data[i] = udp_data[i];
        // 32-bit sum over pseudo-header
    sum = 0;
    etherSumWords(ip->sourceIp, 8);
    tmp_int = ip->protocol;
    sum += (tmp_int & 0xff) << 8;
    etherSumWords(&udp->length, 2);
    // add udp header except crc
    etherSumWords(udp, 6);
    etherSumWords(&udp->data, udp_size);
    udp->check = getEtherChecksum();

    // send packet with size = ether + udp hdr + ip header + udp_size
    etherPutPacket((uint8_t*)ether, 22 + ((ip->rev_size & 0xF) * 4) + udp_size);
}

uint16_t etherGetId()
{
    return htons(sequenceId);
}

void etherIncId()
{
    sequenceId++;
}

// Determines if the IP address is valid
bool etherIsValidIp()
{
    return ipv4Address[0] || ipv4Address[1] || ipv4Address[2] || ipv4Address[3];
}

// Sets IP address
void etherSetIpAddress(uint8_t a, uint8_t b,  uint8_t c, uint8_t d)
{
    ipv4Address[0] = a;
    ipv4Address[1] = b;
    ipv4Address[2] = c;
    ipv4Address[3] = d;
}


// Determines whether packet is TCP datagram
// Must be an IP packet
uint8_t etherIsTcp(uint8_t data[])
{
    uint8_t ok;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    ok = (ip->protocol == 0x06);
    if (ok)
    {
        ok = (tcp->hlengthf!=0x1050);  //to not send tcp ack if we receive and ack
    }
    return ok;
}

uint8_t etherIsTcpAck(uint8_t data[])
{
    uint8_t ok;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    ok = (ip->protocol == 0x06);
    if (ok)
    {
        ok = (tcp->hlengthf==0x1050);  //to consider the ack message inorder to parse the packet
    }
    return ok;
}

uint8_t etherIsFinAck(uint8_t data[])
{
    uint8_t ok;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    ok = (ip->protocol == 0x06);
    if (ok)
    {
        ok = (tcp->hlengthf==0x1150);       //to process an ack message to (fin,ack)
    }
    return ok;
}

void ethersendTcpSyn()
{
    uint8_t i, tmp;
    uint16_t tmp_int;

    ether->sourceAddress[0]=0x00;
    ether->sourceAddress[1]=0x01;
    ether->sourceAddress[2]=0x02;
    ether->sourceAddress[3]=0x03;
    ether->sourceAddress[4]=0x04;
    ether->sourceAddress[5]=0x05;
    ether->destAddress[0]=0xC8;
    ether->destAddress[1]=0xD3;
    ether->destAddress[2]=0xFF;
    ether->destAddress[3]=0xD3;
    ether->destAddress[4]=0x57;
    ether->destAddress[5]=0x59;
    ether->frameType=0x0008;

    ip->sourceIp[0]=0xC0;
    ip->sourceIp[1]=0xA8;
    ip->sourceIp[2]=0x89;
    ip->sourceIp[3]=0xCA;
    ip->destIp[0]=0xD0;
    ip->destIp[1]=0x4C;
    ip->destIp[2]=0xE1;
    ip->destIp[3]=0x49;

    ip->rev_size=0x45;
    ip->typeOfService=0x00;
    ip->id=0x8244;
    ip->flagsAndOffset=0x0040;
    ip->ttl=0x80;
    ip->protocol=0x06;
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();

    tcp->sourcePort = 0x98C3;
    tcp->destPort = 0x5000;

    tcp->ackNum= 0;
    tcp->seqNum += 1 ;
    tcp->hlengthf=0x0280;
    tcp->windowSize=0xF0FA;
    tcp->UrgentPointer=0x0000;
    tcp->options1=0xB4050402;
    tcp->options2=0x08030301;
    tcp->bal=0x02040101;
    // adjust lengths
    ip->length = htons(((ip->rev_size & 0xF) * 4) + 8 + 24);
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();
    // 32-bit sum over pseudo-header
    sum = 0;
    etherSumWords(ip->sourceIp, 8);
    tmp_int = ip->protocol;
    sum += (tmp_int & 0xff) << 8;
    etherSumWords(&tcp->hlengthf, 1);
    // add udp header except crc
    etherSumWords(tcp, 16);
    etherSumWords(&tcp->UrgentPointer, 14);
    tcp->check = getEtherChecksum();

    // send packet with tcp syn
    etherPutPacket((uint8_t*)ether, 22 + ((ip->rev_size & 0xF) * 4) + 24);
}


void etherSendTcpAckback(uint8_t data[])
{
    uint8_t i, tmp;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));
    // swap source and destination fields

    for (i = 0; i < 6; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }


    for (i = 0; i < 4; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }

    ip->id=0x8344;
    ip->ttl=0x80;
    // set source port of resp will be dest port of req
    // dest port of resp will be left at source port of req
    // unusual nomenclature, but this allows a different tx
    // and rx port on other machine
    tmp_int=tcp->sourcePort;
    tcp->sourcePort = tcp->destPort;
    tcp->destPort=tmp_int;

    uint32_t temp = tcp->ackNum;
    tcp->ackNum=tcp->seqNum;
    tcp->ackNum= tcp->ackNum + 0x01000000;
    tcp->seqNum= temp;
    tcp->hlengthf = 0x1050;
    // adjust lengths
    ip->length = htons(((ip->rev_size & 0xF) * 4) + 8 + 12);

    tcp->windowSize=0x0001;
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();

    // 32-bit sum over pseudo-header
    sum = 0;
    etherSumWords(ip->sourceIp, 8);
    tmp_int = ip->protocol;
    sum += (tmp_int & 0xff) << 8;
    etherSumWords(&tcp->hlengthf, 2);
    // add udp header except crc
    etherSumWords(tcp, 16);
    etherSumWords(&tcp->UrgentPointer, 2);
    tcp->check = getEtherChecksum();

    // send packet with size = ether + ip header + tcp
    etherPutPacket((uint8_t*)ether, 22 + ((ip->rev_size & 0xF) * 4) + 12);
}

void etherSendhttpget(uint8_t data[])
{
    char message[] = "GET /xml/current_obs/KDFW.xml HTTP/1.1\r\nHost: w1.weather.gov\r\nUser-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/65.0.3325.181 Safari/537.36\r\nAccept: text/html,*/*\r\n\r\n";
    char* ptr = (void*)&tcp->options1;
    uint8_t httpLen = strlen(message);
    uint8_t i, tmp;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));
    for(i=0;i<httpLen;i++)
        *ptr++ = message[i];

    ether->sourceAddress[0]=0x00;
    ether->sourceAddress[1]=0x01;
    ether->sourceAddress[2]=0x02;
    ether->sourceAddress[3]=0x03;
    ether->sourceAddress[4]=0x04;
    ether->sourceAddress[5]=0x05;
    ether->destAddress[0]=0xC8;
    ether->destAddress[1]=0xD3;
    ether->destAddress[2]=0xFF;
    ether->destAddress[3]=0xD3;
    ether->destAddress[4]=0x57;
    ether->destAddress[5]=0x59;
    ether->frameType=0x0008;

    ip->sourceIp[0]=0xC0;
    ip->sourceIp[1]=0xA8;
    ip->sourceIp[2]=0x89;
    ip->sourceIp[3]=0xCA;
    ip->destIp[0]=0xD0;
    ip->destIp[1]=0x4C;
    ip->destIp[2]=0xE1;
    ip->destIp[3]=0x49;
    ip->rev_size=0x45;
    ip->typeOfService=0x00;
    ip->id=0x8544;
    ip->ttl=0x80;

    tcp->hlengthf = 0x1850;
    // adjust lengths
    ip->length = htons(((ip->rev_size & 0xF) * 4) + 8 + 12 + httpLen);

    tcp->windowSize=0x0001;

    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();

        // 32-bit sum over pseudo-header
    sum = 0;
    etherSumWords(ip->sourceIp, 8);
    tmp_int = ip->protocol;
    sum += (tmp_int & 0xff) << 8;
    etherSumWords(&tcp->hlengthf, 2);
    // add udp header except crc
    etherSumWords(tcp, 16);
    etherSumWords(&tcp->UrgentPointer, 2);
    etherSumWords(tcp->data, httpLen);

    tcp->check = getEtherChecksum();

    // send packet with http data
    etherPutPacket((uint8_t*)ether, 22 + ((ip->rev_size & 0xF) * 4) + 12 + strlen(message));
}

void etherSendHttpAckback(uint8_t data[])
{
    uint8_t i, tmp;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));
    // swap source and destination fields


    for (i = 0; i < 6; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }


    for (i = 0; i < 4; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }

    ip->id=0x8344;
    ip->ttl=0x80;
    // set source port of resp will be dest port of req
    // dest port of resp will be left at source port of req
    // unusual nomenclature, but this allows a different tx
    // and rx port on other machine
    tmp_int=tcp->sourcePort;
    tcp->sourcePort = tcp->destPort;
    tcp->destPort=tmp_int;

    uint32_t nextseqNum;
    uint16_t tcpsegLen;
    tcpsegLen = (enc28j60->size)-20-20-14-4;
    tcpsegLen = htons(tcpsegLen);
    uint32_t segLen=0;
    segLen = (0xFFFF & tcpsegLen) << 16;
    nextseqNum = tcp->seqNum + segLen;
    tcp->seqNum = tcp->ackNum;
    tcp->ackNum = nextseqNum;

    tcp->hlengthf = 0x1050;
    ip->length = htons(((ip->rev_size & 0xF) * 4) + 8 + 12);

    tcp->windowSize=0x0001;
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();

    sum = 0;
    etherSumWords(ip->sourceIp, 8);
    tmp_int = ip->protocol;
    sum += (tmp_int & 0xff) << 8;
    etherSumWords(&tcp->hlengthf, 2);
    // add udp header except crc
    etherSumWords(tcp, 16);
    etherSumWords(&tcp->UrgentPointer, 2);
    tcp->check = getEtherChecksum();

    // send packet with http ack
    etherPutPacket((uint8_t*)ether, 22 + ((ip->rev_size & 0xF) * 4) + 12);
}

uint8_t parsetemp(uint8_t data[])
{
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    uint16_t i=0;
    uint8_t j=0;
    for(i=0; i<1530; i++)
    {
        if(tcp->data[i]=='t')
        {
            if(tcp->data[++i]=='e')
            {
                if(tcp->data[++i]=='m')
                {
                    if(tcp->data[++i]=='p')
                    {
                        if(tcp->data[++i]==0x5F)
                        {
                            if(tcp->data[++i]=='f')
                            {
                                i++;
                                for(j=0; j<2; j++)
                                {
                                    parsedData[j] = tcp->data[++i];
                                }
                                j=0;
                                putsUart0("Temperature in Fahrenheit\t");
                                putsUart0(parsedData);
                                putsUart0("\r\n");
                                temp=(atoi(parsedData));
                                memset(parsedData,0,sizeof(parsedData));
                                return temp;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
}

uint8_t parsehum(uint8_t data[])
{
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    uint16_t i=0;
    uint8_t j=0;
    for(i=0; i<1530; i++)
    {
        if(tcp->data[i]=='h')
        {
            if(tcp->data[++i]=='u')
            {
                if(tcp->data[++i]=='m')
                {
                    if(tcp->data[++i]=='i')
                    {
                        if(tcp->data[++i]=='d')
                        {
                            if(tcp->data[++i]=='i')
                            {
                                if(tcp->data[++i]=='t')
                                {
                                    if(tcp->data[++i]=='y')
                                    {
                                        if(tcp->data[++i]==0x3E)
                                        {
                                            for(j=0; j<2; j++)
                                            {
                                                parsedData[j] = tcp->data[++i];
                                            }
                                            j=0;
                                            /*if((parsedData[0] && parsedData[1])==0x00)
                                            {
                                                return hum;
                                            }*/
                                            putsUart0("Humidity is\t");
                                            putsUart0(parsedData);
                                            putsUart0("\r\n");
                                            hum=atoi(parsedData);
                                            memset(parsedData,0,sizeof(parsedData));
                                            return hum;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    i=0;
}

uint8_t parsehum1(uint8_t data[])
{
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    uint16_t i=0;
    uint8_t j=0;
    for(i=0; i<1530; i++)
    {
        if(tcp->data[i]==0x3C)
        {
            if(tcp->data[++i]==0x2F)
            {
                if(tcp->data[++i]=='r')
                {
                    if(tcp->data[++i]=='e')
                    {
                        if(tcp->data[++i]=='l')
                        {
                            if(tcp->data[++i]=='a')
                            {
                                if(tcp->data[++i]=='t')
                                {
                                    if(tcp->data[++i]=='i')
                                    {
                                        if(tcp->data[++i]=='v')
                                        {
                                            if(tcp->data[++i]=='e')
                                            {
                                                i=i-11;
                                                for(j=0; j<2; j++)
                                                {
                                                    parsedData[j] = tcp->data[i++];
                                                }
                                                j=0;
                                                /*if((parsedData[0] && parsedData[1])==0x00)
                                                {
                                                    return hum;
                                                }*/
                                                putsUart0("Humidity1 is\t");
                                                putsUart0(parsedData);
                                                putsUart0("\r\n");
                                                hum=atoi(parsedData);
                                                memset(parsedData,0,sizeof(parsedData));
                                                return hum;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    i=0;
}

void finackcomplete(uint8_t data[])
{
    uint8_t i, tmp;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));

    ip->id=0x1445;
    ip->flagsAndOffset=0x0040;
    ip->ttl=0x80;
    tcp->seqNum = tcp->seqNum;
    tcp->ackNum = tcp->ackNum;
    tcp->hlengthf = 0x1150;
    ip->length = htons(((ip->rev_size & 0xF) * 4) + 8+ 12);
    tcp->windowSize=0x0001;
    tcp->UrgentPointer=0x0000;
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();

    sum = 0;
    etherSumWords(ip->sourceIp, 8);
    tmp_int = ip->protocol;
    sum += (tmp_int & 0xff) << 8;
    etherSumWords(&tcp->hlengthf, 2);
    // add udp header except crc
    etherSumWords(tcp, 16);
    etherSumWords(&tcp->UrgentPointer, 2);
    tcp->check = getEtherChecksum();

    // send packet with (fin,ack)
    etherPutPacket((uint8_t*)ether, 22 + ((ip->rev_size & 0xF) * 4) + 12);
}