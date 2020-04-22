// ETH0 Library
// Jason Losh
// Modified by Saeed Jassani

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL w/ ENC28J60
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// ENC28J60 Ethernet controller on SPI0
//   MOSI (SSI0Tx) on PA5
//   MISO (SSI0Rx) on PA4
//   SCLK (SSI0Clk) on PA2
//   ~CS (SW controlled) on PA3
//   WOL on PB3
//   INT on PC6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <eth0.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "gpio.h"
#include "spi0.h"
#include "eprom.h"

// Pins
#define CS PORTA,3
#define WOL PORTB,3
#define INT PORTC,6

// Ether registers
#define ERDPTL      0x00
#define ERDPTH      0x01
#define EWRPTL      0x02
#define EWRPTH      0x03
#define ETXSTL      0x04
#define ETXSTH      0x05
#define ETXNDL      0x06
#define ETXNDH      0x07
#define ERXSTL      0x08
#define ERXSTH      0x09
#define ERXNDL      0x0A
#define ERXNDH      0x0B
#define ERXRDPTL    0x0C
#define ERXRDPTH    0x0D
#define ERXWRPTL    0x0E
#define ERXWRPTH    0x0F
#define EIE         0x1B
#define EIR         0x1C
#define RXERIF  0x01
#define TXERIF  0x02
#define TXIF    0x08
#define PKTIF   0x40
#define ESTAT       0x1D
#define CLKRDY  0x01
#define TXABORT 0x02
#define ECON2       0x1E
#define PKTDEC  0x40
#define ECON1       0x1F
#define RXEN    0x04
#define TXRTS   0x08
#define ERXFCON     0x38
#define EPKTCNT     0x39
#define MACON1      0x40
#define MARXEN  0x01
#define RXPAUS  0x04
#define TXPAUS  0x08
#define MACON2      0x41
#define MARST   0x80
#define MACON3      0x42
#define FULDPX  0x01
#define FRMLNEN 0x02
#define TXCRCEN 0x10
#define PAD60   0x20
#define MACON4      0x43
#define MABBIPG     0x44
#define MAIPGL      0x46
#define MAIPGH      0x47
#define MACLCON1    0x48
#define MACLCON2    0x49
#define MAMXFLL     0x4A
#define MAMXFLH     0x4B
#define MICMD       0x52
#define MIIRD   0x01
#define MIREGADR    0x54
#define MIWRL       0x56
#define MIWRH       0x57
#define MIRDL       0x58
#define MIRDH       0x59
#define MAADR1      0x60
#define MAADR0      0x61
#define MAADR3      0x62
#define MAADR2      0x63
#define MAADR5      0x64
#define MAADR4      0x65
#define MISTAT      0x6A
#define MIBUSY  0x01
#define ECOCON      0x75

// Ether phy registers
#define PHCON1      0x00
#define PDPXMD 0x0100
#define PHSTAT1     0x01
#define LSTAT  0x0400
#define PHCON2      0x10
#define HDLDIS 0x0100
#define PHLCON      0x14

// ------------------------------------------------------------------------------
//  Structures
// ------------------------------------------------------------------------------

// This M4F is little endian (TI hardwired it this way)
// Network byte order is big endian
// Must interpret uint16_t in reverse order

typedef struct _enc28j60Frame // 4-bytes
{
	uint16_t size;
	uint16_t status;
	uint8_t data;
} enc28j60Frame;

typedef struct _etherFrame // 14-bytes
{
	uint8_t destAddress[6];
	uint8_t sourceAddress[6];
	uint16_t frameType;
	uint8_t data;
} etherFrame;

typedef struct _ipFrame // minimum 20 bytes
{
	uint8_t revSize;
	uint8_t typeOfService;
	uint16_t length;
	uint16_t id;
	uint16_t flagsAndOffset;
	uint8_t ttl;
	uint8_t protocol;
	uint16_t headerChecksum;
	uint8_t sourceIp[4];
	uint8_t destIp[4];
} ipFrame;

typedef struct _icmpFrame {
	uint8_t type;
	uint8_t code;
	uint16_t check;
	uint16_t id;
	uint16_t seq_no;
	uint8_t data;
} icmpFrame;

typedef struct _arpFrame {
	uint16_t hardwareType;
	uint16_t protocolType;
	uint8_t hardwareSize;
	uint8_t protocolSize;
	uint16_t op;
	uint8_t sourceAddress[6];
	uint8_t sourceIp[4];
	uint8_t destAddress[6];
	uint8_t destIp[4];
} arpFrame;

typedef struct _udpFrame // 8 bytes
{
	uint16_t sourcePort;
	uint16_t destPort;
	uint16_t length;
	uint16_t check;
	uint8_t data;
} udpFrame;

typedef struct _tcpFrame // 8 bytes
{
	uint16_t sourcePort;
	uint16_t destPort;
	uint32_t sequenceNumber;
	uint32_t ackNumber;
	uint16_t headerLength;
	uint16_t windowSize;
	uint16_t checksum;
	uint16_t urgent;
	uint8_t options[0];
	uint8_t data;
} tcpFrame;

typedef struct _dhcpFrame {
	uint8_t op;
	uint8_t htype;
	uint8_t hlen;
	uint8_t hops;
	uint32_t xid;
	uint16_t secs;
	uint16_t flags;
	uint8_t ciaddr[4];
	uint8_t yiaddr[4];
	uint8_t siaddr[4];
	uint8_t giaddr[4];
	uint8_t chaddr[16];
	uint8_t data[192];
	uint32_t magicCookie;
	uint8_t options[0];
} dhcpFrame;

// ------------------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------------------

uint8_t nextPacketLsb = 0x00;
uint8_t nextPacketMsb = 0x00;
uint8_t sequenceId = 1;
uint32_t sum;

extern bool dhcpEnabled = false;

uint8_t macAddress[HW_ADD_LENGTH] = { 2, 3, 4, 5, 6, 136 };
uint8_t ipAddress[IP_ADD_LENGTH] = { 0, 0, 0, 0 };
uint8_t tempIpAddress[IP_ADD_LENGTH] = { 0, 0, 0, 0 };
uint8_t ipSubnetMask[IP_ADD_LENGTH] = { 255, 255, 255, 0 };
uint8_t ipGwAddress[IP_ADD_LENGTH] = { 0, 0, 0, 0 };
uint8_t dnsAddress[IP_ADD_LENGTH] = { 0, 0, 0, 0 };

uint8_t serverIpAddress[IP_ADD_LENGTH] = { 0, 0, 0, 0 };
uint8_t serverMacAddress[HW_ADD_LENGTH] = { 0, 0, 0, 0, 0, 0 };

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Buffer is configured as follows
// Receive buffer starts at 0x0000 (bottom 6666 bytes of 8K space)
// Transmit buffer at 01A0A (top 1526 bytes of 8K space)

void etherCsOn() {
	setPinValue(CS, 0);
	__asm (" NOP");
	// allow line to settle
	__asm (" NOP");
	__asm (" NOP");
	__asm (" NOP");
}

void etherCsOff() {
	setPinValue(CS, 1);
}

void etherWriteReg(uint8_t reg, uint8_t data) {
	etherCsOn();
	writeSpi0Data(0x40 | (reg & 0x1F));
	readSpi0Data();
	writeSpi0Data(data);
	readSpi0Data();
	etherCsOff();
}

uint8_t etherReadReg(uint8_t reg) {
	uint8_t data;
	etherCsOn();
	writeSpi0Data(0x00 | (reg & 0x1F));
	readSpi0Data();
	writeSpi0Data(0);
	data = readSpi0Data();
	etherCsOff();
	return data;
}

void etherSetReg(uint8_t reg, uint8_t mask) {
	etherCsOn();
	writeSpi0Data(0x80 | (reg & 0x1F));
	readSpi0Data();
	writeSpi0Data(mask);
	readSpi0Data();
	etherCsOff();
}

void etherClearReg(uint8_t reg, uint8_t mask) {
	etherCsOn();
	writeSpi0Data(0xA0 | (reg & 0x1F));
	readSpi0Data();
	writeSpi0Data(mask);
	readSpi0Data();
	etherCsOff();
}

void etherSetBank(uint8_t reg) {
	etherClearReg(ECON1, 0x03);
	etherSetReg(ECON1, reg >> 5);
}

void etherWritePhy(uint8_t reg, uint16_t data) {
	etherSetBank(MIREGADR);
	etherWriteReg(MIREGADR, reg);
	etherWriteReg(MIWRL, data & 0xFF);
	etherWriteReg(MIWRH, (data >> 8) & 0xFF);
}

uint16_t etherReadPhy(uint8_t reg) {
	uint16_t data, dataH;
	etherSetBank(MIREGADR);
	etherWriteReg(MIREGADR, reg);
	etherWriteReg(MICMD, MIIRD);
	waitMicrosecond(11);
	etherSetBank(MISTAT);
	while ((etherReadReg(MISTAT) & MIBUSY) != 0)
		;
	etherSetBank(MICMD);
	etherWriteReg(MICMD, 0);
	data = etherReadReg(MIRDL);
	dataH = etherReadReg(MIRDH);
	data |= (dataH << 8);
	return data;
}

void etherWriteMemStart() {
	etherCsOn();
	writeSpi0Data(0x7A);
	readSpi0Data();
}

void etherWriteMem(uint8_t data) {
	writeSpi0Data(data);
	readSpi0Data();
}

void etherWriteMemStop() {
	etherCsOff();
}

void etherReadMemStart() {
	etherCsOn();
	writeSpi0Data(0x3A);
	readSpi0Data();
}

uint8_t etherReadMem() {
	writeSpi0Data(0);
	return readSpi0Data();
}

void etherReadMemStop() {
	etherCsOff();
}

// Initializes ethernet device
// Uses order suggested in Chapter 6 of datasheet except 6.4 OST which is first here
void etherInit(uint16_t mode) {
	// Initialize SPI0
	initSpi0(USE_SSI0_RX);
	setSpi0BaudRate(4e6, 40e6);
	setSpi0Mode(0, 0);

	// Enable clocks
	enablePort(PORTA);
	enablePort(PORTB);
	enablePort(PORTC);

	// Configure pins for ethernet module
	selectPinPushPullOutput(CS);
	selectPinDigitalInput(WOL);
	selectPinDigitalInput(INT);

	// make sure that oscillator start-up timer has expired
	while ((etherReadReg(ESTAT) & CLKRDY) == 0) {
	}

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
	etherWriteReg(ERXFCON, (mode | ETHER_CHECKCRC) & 0xFF);

	// bring mac out of reset
	etherSetBank(MACON2);
	etherWriteReg(MACON2, 0);

	// enable mac rx, enable pause control for full duplex
	etherWriteReg(MACON1, TXPAUS | RXPAUS | MARXEN);

	// enable padding to 60 bytes (no runt packets)
	// add crc to tx packets, set full or half duplex
	if ((mode & ETHER_FULLDUPLEX) != 0)
		etherWriteReg(MACON3, FULDPX | FRMLNEN | TXCRCEN | PAD60);
	else etherWriteReg(MACON3, FRMLNEN | TXCRCEN | PAD60);

	// leave MACON4 as reset

	// set maximum rx packet size
	etherWriteReg(MAMXFLL, LOBYTE(1518));
	etherWriteReg(MAMXFLH, HIBYTE(1518));

	// set back-to-back inter-packet gap to 9.6us
	if ((mode & ETHER_FULLDUPLEX) != 0)
		etherWriteReg(MABBIPG, 0x15);
	else etherWriteReg(MABBIPG, 0x12);

	// set non-back-to-back inter-packet gap registers
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
	else etherWritePhy(PHCON1, 0);

	// disable phy loopback if in half-duplex mode
	etherWritePhy(PHCON2, HDLDIS);

	// Flash LEDA and LEDB
	etherWritePhy(PHLCON, 0x0880);
	waitMicrosecond(100000);

	// set LEDA (link status) and LEDB (tx/rx activity)
	// stretch LED on to 40ms (default)
	etherWritePhy(PHLCON, 0x0472);
	// enable reception
	etherSetReg(ECON1, RXEN);

	// Read DHCP state from EPROM
	uint32_t tmp = readEeprom(0);
	dhcpEnabled = tmp > 0;

	if (!dhcpEnabled) {
		getDetailsFromEprom();
	}
}

// Returns true if link is up
bool etherIsLinkUp() {
	return (etherReadPhy(PHSTAT1) & LSTAT) != 0;
}

// Returns TRUE if packet received
bool etherIsDataAvailable() {
	return ((etherReadReg(EIR) & PKTIF) != 0);
}

// Returns true if rx buffer overflowed after correcting the problem
bool etherIsOverflow() {
	bool err;
	err = (etherReadReg(EIR) & RXERIF) != 0;
	if (err)
		etherClearReg(EIR, RXERIF);
	return err;
}

// Returns up to max_size characters in data buffer
// Returns number of bytes copied to buffer
// Contents written are 16-bit size, 16-bit status, payload excl crc
uint16_t etherGetPacket(uint8_t packet[], uint16_t maxSize) {
	uint16_t i = 0, size, tmp16, status;

	// enable read from FIFO buffers
	etherReadMemStart();

	// get next packet information
	nextPacketLsb = etherReadMem();
	nextPacketMsb = etherReadMem();

	// calc size
	// don't return crc, instead return size + status, so size is correct
	size = etherReadMem();
	tmp16 = etherReadMem();
	size |= (tmp16 << 8);

	// get status (currently unused)
	status = etherReadMem();
	tmp16 = etherReadMem();
	status |= (tmp16 << 8);

	// copy data
	if (size > maxSize)
		size = maxSize;
	while (i < size)
		packet[i++] = etherReadMem();

	// end read from FIFO buffers
	etherReadMemStop();

	// advance read pointer
	etherSetBank(ERXRDPTL);
	etherWriteReg(ERXRDPTL, nextPacketLsb); // hw ptr
	etherWriteReg(ERXRDPTH, nextPacketMsb);
	etherWriteReg(ERDPTL, nextPacketLsb);   // dma rd ptr
	etherWriteReg(ERDPTH, nextPacketMsb);

	// decrement packet counter so that PKTIF is maintained correctly
	etherSetReg(ECON2, PKTDEC);

	return size;
}

// Writes a packet
bool etherPutPacket(uint8_t packet[], uint16_t size) {
	uint16_t i;

	// clear out any tx errors
	if ((etherReadReg(EIR) & TXERIF) != 0) {
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
		etherWriteMem(packet[i]);

	// stop write
	etherWriteMemStop();

	// request transmit
	etherWriteReg(ETXSTL, LOBYTE(0x1A0A));
	etherWriteReg(ETXSTH, HIBYTE(0x1A0A));
	etherWriteReg(ETXNDL, LOBYTE(0x1A0A + size));
	etherWriteReg(ETXNDH, HIBYTE(0x1A0A + size));
	etherClearReg(EIR, TXIF);
	etherSetReg(ECON1, TXRTS);

	// wait for completion
	while ((etherReadReg(ECON1) & TXRTS) != 0)
		;

	// determine success
	return ((etherReadReg(ESTAT) & TXABORT) == 0);
}

// Calculate sum of words
// Must use getEtherChecksum to complete 1's compliment addition
void etherSumWords(void* data, uint16_t sizeInBytes) {
	uint8_t* pData = (uint8_t*) data;
	uint16_t i;
	uint8_t phase = 0;
	uint16_t data_temp;
	for (i = 0; i < sizeInBytes; i++) {
		if (phase) {
			data_temp = *pData;
			sum += data_temp << 8;
		} else sum += *pData;
		phase = 1 - phase;
		pData++;
	}
}

// Completes 1's compliment addition by folding carries back into field
uint16_t getEtherChecksum() {
	uint16_t result;
	// this is based on rfc1071
	while ((sum >> 16) > 0)
		sum = (sum & 0xFFFF) + (sum >> 16);
	result = sum & 0xFFFF;
	return ~result;
}

void etherCalcIpChecksum(ipFrame* ip) {
	// 32-bit sum over ip header
	sum = 0;
	etherSumWords(&ip->revSize, 10);
	etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);
	ip->headerChecksum = getEtherChecksum();
}

// Converts from host to network order and vice versa
uint16_t htons(uint16_t value) {
	return ((value & 0xFF00) >> 8) + ((value & 0x00FF) << 8);
}
#define ntohs htons

// Snippet from https://forum.arduino.cc/index.php?topic=44867.0
uint32_t htols(uint32_t value) {
	uint32_t tmp_a = (value & 0xff000000) >> 24;
	uint32_t tmp_b = (value & 0x00ff0000) >> 8;
	uint32_t tmp_c = (value & 0x0000ff00) << 8;
	uint32_t tmp_d = (value & 0x000000ff) << 24;
	return tmp_d | tmp_c | tmp_b | tmp_a;

}

// Determines whether packet is IP datagram
bool etherIsIp(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	bool ok;
	ok = (ether->frameType == htons(0x0800));
	if (ok) {
		sum = 0;
		etherSumWords(&ip->revSize, (ip->revSize & 0xF) * 4);
		ok = (getEtherChecksum() == 0);
	}
	return ok;
}

// Determines whether packet is unicast to this ip
// Must be an IP packet
bool etherIsIpUnicast(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	uint8_t i = 0;
	bool ok = true;
	while (ok & (i < IP_ADD_LENGTH)) {
		ok = (ip->destIp[i] == ipAddress[i]);
		i++;
	}
	return ok;
}

// Determines whether packet is braodcast to this ip
// Must be an IP packet
bool etherIsIpBroadcast(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	uint8_t i = 0;
	bool ok = true;
	while (ok & (i < IP_ADD_LENGTH)) {
		ok = (ip->destIp[i] == 255);
		i++;
	}
	return ok;
}

// Determines whether packet is ping request
// Must be an IP packet
bool etherIsPingRequest(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	icmpFrame* icmp = (icmpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	return (ip->protocol == 0x01 & icmp->type == 8);
}

// Sends a ping response given the request data
void etherSendPingResponse(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	icmpFrame* icmp = (icmpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	uint8_t i, tmp;
	uint16_t icmp_size;
	// swap source and destination fields
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		tmp = ether->destAddress[i];
		ether->destAddress[i] = ether->sourceAddress[i];
		ether->sourceAddress[i] = tmp;
	}
	for (i = 0; i < IP_ADD_LENGTH; i++) {
		tmp = ip->destIp[i];
		ip->destIp[i] = ip->sourceIp[i];
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
	etherPutPacket(ether, 14 + ntohs(ip->length));
}

// Determines whether packet is ARP
bool etherIsArpRequest(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	arpFrame* arp = (arpFrame*) &ether->data;
	bool ok;
	uint8_t i = 0;
	ok = (ether->frameType == htons(0x0806));
	while (ok & (i < IP_ADD_LENGTH)) {
		ok = (arp->destIp[i] == ipAddress[i]);
		i++;
	}
	if (ok)
		ok = (arp->op == htons(1));
	return ok;
}

// Sends an ARP response given the request data
void etherSendArpResponse(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	arpFrame* arp = (arpFrame*) &ether->data;
	uint8_t i, tmp;
	// set op to response
	arp->op = htons(2);
	// swap source and destination fields
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		arp->destAddress[i] = arp->sourceAddress[i];
		ether->destAddress[i] = ether->sourceAddress[i];
		ether->sourceAddress[i] = arp->sourceAddress[i] = macAddress[i];
	}
	for (i = 0; i < IP_ADD_LENGTH; i++) {
		tmp = arp->destIp[i];
		arp->destIp[i] = arp->sourceIp[i];
		arp->sourceIp[i] = tmp;
	}
	// send packet
	etherPutPacket(ether, 42);
}

// Sends an ARP request
void etherSendArpRequest(uint8_t packet[], uint8_t ip[]) {
	etherFrame* ether = (etherFrame*) packet;
	arpFrame* arp = (arpFrame*) &ether->data;
	uint8_t i;
	// fill ethernet frame
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		ether->destAddress[i] = 0xFF;
		ether->sourceAddress[i] = macAddress[i];
	}
	ether->frameType = 0x0608;
	// fill arp frame
	arp->hardwareType = htons(1);
	arp->protocolType = htons(0x0800);
	arp->hardwareSize = HW_ADD_LENGTH;
	arp->protocolSize = IP_ADD_LENGTH;
	arp->op = htons(1);
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		arp->sourceAddress[i] = macAddress[i];
		arp->destAddress[i] = 0xFF;
	}
	for (i = 0; i < IP_ADD_LENGTH; i++) {
		arp->sourceIp[i] = ipAddress[i];
		arp->destIp[i] = ip[i];
	}
	// send packet
	etherPutPacket(ether, 42);
}

// Determines whether packet is UDP datagram
// Must be an IP packet
bool etherIsUdp(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	udpFrame* udp = (udpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	bool ok;
	uint16_t tmp16;
	ok = (ip->protocol == 0x11);
	if (ok) {
		// 32-bit sum over pseudo-header
		sum = 0;
		etherSumWords(ip->sourceIp, 8);
		tmp16 = ip->protocol;
		sum += (tmp16 & 0xff) << 8;
		etherSumWords(&udp->length, 2);
		// add udp header and data
		etherSumWords(udp, ntohs(udp->length));
		ok = (getEtherChecksum() == 0);
	}
	return ok;
}

// Gets pointer to UDP payload of frame
uint8_t* etherGetUdpData(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	udpFrame* udp = (udpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	return &udp->data;
}

// Send responses to a udp datagram 
// destination port, ip, and hardware address are extracted from provided data
// uses destination port of received packet as destination of this packet
void etherSendUdpResponse(uint8_t packet[], uint8_t* udpData, uint8_t udpSize) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	udpFrame* udp = (udpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	uint8_t *copyData;
	uint8_t i, tmp8;
	uint16_t tmp16;
	// swap source and destination fields
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		tmp8 = ether->destAddress[i];
		ether->destAddress[i] = ether->sourceAddress[i];
		ether->sourceAddress[i] = tmp8;
	}
	for (i = 0; i < IP_ADD_LENGTH; i++) {
		tmp8 = ip->destIp[i];
		ip->destIp[i] = ip->sourceIp[i];
		ip->sourceIp[i] = tmp8;
	}
	// set source port of resp will be dest port of req
	// dest port of resp will be left at source port of req
	// unusual nomenclature, but this allows a different tx
	// and rx port on other machine
	udp->sourcePort = udp->destPort;
	// adjust lengths
	ip->length = htons(((ip->revSize & 0xF) * 4) + 8 + udpSize);
	// 32-bit sum over ip header
	sum = 0;
	etherSumWords(&ip->revSize, 10);
	etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);
	ip->headerChecksum = getEtherChecksum();
	udp->length = htons(8 + udpSize);
	// copy data
	copyData = &udp->data;
	for (i = 0; i < udpSize; i++)
		copyData[i] = udpData[i];
	// 32-bit sum over pseudo-header
	sum = 0;
	etherSumWords(ip->sourceIp, 8);
	tmp16 = ip->protocol;
	sum += (tmp16 & 0xff) << 8;
	etherSumWords(&udp->length, 2);
	// add udp header except crc
	etherSumWords(udp, 6);
	etherSumWords(&udp->data, udpSize);
	udp->check = getEtherChecksum();

	// send packet with size = ether + udp hdr + ip header + udp_size
	etherPutPacket(ether, 22 + ((ip->revSize & 0xF) * 4) + udpSize);
}

uint16_t etherGetId() {
	return htons(sequenceId);
}

void etherIncId() {
	sequenceId++;
}

// Determines if the IP address is valid
bool etherIsIpValid() {
	return ipAddress[0] || ipAddress[1] || ipAddress[2] || ipAddress[3];
}

// Sets IP address
void etherSetIpAddress(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3) {
	ipAddress[0] = ip0;
	ipAddress[1] = ip1;
	ipAddress[2] = ip2;
	ipAddress[3] = ip3;

	uint32_t ip = 0;
	ip |= (ip0 & 0x000000ff) << 24;
	ip |= (ip1 & 0x000000ff) << 16;
	ip |= (ip2 & 0x000000ff) << 8;
	ip |= (ip3 & 0x000000ff);
	writeEeprom(1, ip);
}

// Gets IP address
void etherGetIpAddress(uint8_t ip[4]) {
	uint8_t i;
	for (i = 0; i < 4; i++)
		ip[i] = ipAddress[i];
}

// Sets IP address to 0.0.0.0 - Used when lease expires
void etherSetIpAddressToZeroes() {
	uint8_t i;
	for (i = 0; i < 4; i++)
		ipAddress[i] = 0;
}

// Sets IP gateway address
void etherSetIpGatewayAddress(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3) {
	ipGwAddress[0] = ip0;
	ipGwAddress[1] = ip1;
	ipGwAddress[2] = ip2;
	ipGwAddress[3] = ip3;

	uint32_t ip = 0;
	ip |= ip0 << 24;
	ip |= ip1 << 16;
	ip |= ip2 << 8;
	ip |= ip3;
	writeEeprom(2, ip);
}

// Gets IP gateway address
void etherGetIpGatewayAddress(uint8_t ip[4]) {
	uint8_t i;
	for (i = 0; i < 4; i++)
		ip[i] = ipGwAddress[i];
}

// Sets DNS address
void etherSetDNSAddress(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3) {
	ipGwAddress[0] = ip0;
	ipGwAddress[1] = ip1;
	ipGwAddress[2] = ip2;
	ipGwAddress[3] = ip3;

	uint32_t ip = 0;
	ip |= ip0 << 24;
	ip |= ip1 << 16;
	ip |= ip2 << 8;
	ip |= ip3;
	writeEeprom(3, ip);
}

// Gets DNS address
void etherGetDNSAddress(uint8_t ip[4]) {
	uint8_t i;
	for (i = 0; i < 4; i++)
		ip[i] = dnsAddress[i];
}

// Sets IP subnet mask
void etherSetIpSubnetMask(uint8_t mask0, uint8_t mask1, uint8_t mask2, uint8_t mask3) {
	ipSubnetMask[0] = mask0;
	ipSubnetMask[1] = mask1;
	ipSubnetMask[2] = mask2;
	ipSubnetMask[3] = mask3;

	uint32_t ip = 0;
	ip |= mask0 << 24;
	ip |= mask1 << 16;
	ip |= mask2 << 8;
	ip |= mask3;
	writeEeprom(4, ip);
}

// Gets IP subnet mask
void etherGetIpSubnetMask(uint8_t mask[4]) {
	uint8_t i;
	for (i = 0; i < 4; i++)
		mask[i] = ipSubnetMask[i];
}

// Sets MAC address
void etherSetMacAddress(uint8_t mac0, uint8_t mac1, uint8_t mac2, uint8_t mac3, uint8_t mac4, uint8_t mac5) {
	macAddress[0] = mac0;
	macAddress[1] = mac1;
	macAddress[2] = mac2;
	macAddress[3] = mac3;
	macAddress[4] = mac4;
	macAddress[5] = mac5;
}

// Gets MAC address
void etherGetMacAddress(uint8_t mac[6]) {
	uint8_t i;
	for (i = 0; i < 6; i++)
		mac[i] = macAddress[i];
}

void getDetailsFromEprom() {

	// Get IP Address
	uint32_t tmp = readEeprom(1);

	ipAddress[0] = (tmp & 0xff000000) >> 24;
	ipAddress[1] = (tmp & 0x00ff0000) >> 16;
	ipAddress[2] = (tmp & 0x0000ff00) >> 8;
	ipAddress[3] = (tmp & 0x000000ff);

	// Get GW Address
	tmp = readEeprom(2);

	ipGwAddress[0] = (tmp & 0xff000000) >> 24;
	ipGwAddress[1] = (tmp & 0x00ff0000) >> 16;
	ipGwAddress[2] = (tmp & 0x0000ff00) >> 8;
	ipGwAddress[3] = (tmp & 0x000000ff);

	// Get DNS Address
	tmp = readEeprom(3);

	dnsAddress[0] = (tmp & 0xff000000) >> 24;
	dnsAddress[1] = (tmp & 0x00ff0000) >> 16;
	dnsAddress[2] = (tmp & 0x0000ff00) >> 8;
	dnsAddress[3] = (tmp & 0x000000ff);

	// Get SN Address
	tmp = readEeprom(4);

	ipSubnetMask[0] = (tmp & 0xff000000) >> 24;
	ipSubnetMask[1] = (tmp & 0x00ff0000) >> 16;
	ipSubnetMask[2] = (tmp & 0x0000ff00) >> 8;
	ipSubnetMask[3] = (tmp & 0x000000ff);
}

// DHCP Functions
// Enable or disable DHCP mode
void etherEnableDhcpMode() {
	dhcpEnabled = true;
	writeEeprom(0, 1);
}

void etherDisableDhcpMode() {
	dhcpEnabled = false;
	writeEeprom(0, 0x000);
}

bool etherIsDhcpEnabled() {
	return dhcpEnabled;
}

// Send DHCP Packets. packet_type - 1 for Discovery, 3 for Request, 4 for Decline, 5 for Renew, 6 for Rebind, 7 for release
void etherSendDhcpPacket(uint8_t packet[], uint8_t packet_type) {

	etherFrame* ether = (etherFrame*) packet;
	uint8_t i;

	for (i = 0; i < HW_ADD_LENGTH; i++) {
		// DHCP Renew and Release are unicast
		if (packet_type == 5 || packet_type == 7) {
			ether->destAddress[i] = serverMacAddress[i];
		} else {
			ether->destAddress[i] = 255;
		}
		ether->sourceAddress[i] = macAddress[i];
	}
	ether->frameType = htons(0x0800);

	uint16_t tmp16;

	ipFrame* ip = (ipFrame*) &ether->data;
	ip->revSize = 0x45;
	ip->typeOfService = 0;
	ip->id = 0;
	ip->flagsAndOffset = 0;
	ip->ttl = TTL;
	ip->protocol = 17;
	for (i = 0; i < IP_ADD_LENGTH; i++) {
		// DHCP Renew and Release are unicast
		if (packet_type == 5 || packet_type == 7) {
			ip->sourceIp[i] = ipAddress[i];
			ip->destIp[i] = serverIpAddress[i];
		} else {
			ip->sourceIp[i] = 0;
			ip->destIp[i] = 255;
		}
	}

	udpFrame* udp = (udpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	udp->sourcePort = htons(68);
	udp->destPort = htons(67);

	dhcpFrame* dhcp = (dhcpFrame*) &udp->data;
	dhcp->op = 1;
	dhcp->htype = 0x01;
	dhcp->hlen = 6;
	dhcp->hops = 0;
	dhcp->xid = 0;
	dhcp->secs = 0;

	// DHCP Renew and Release are unicast
	if (packet_type == 5 || packet_type == 7) {
		dhcp->flags = 0;
	} else {
		dhcp->flags = htons(0x8000);
	}

	dhcp->magicCookie = htols(0x63825363);

	// Save options 51 and 54 from DHCP Offer (to be used in DHCP Request)
	uint8_t t1, t2, t3, t4, t5, t6, t7, t8;
	if (packet_type == 3) {
		uint16_t opSiz = htons(udp->length - 248);
		uint8_t* o54 = getOption(dhcp->options, 54, opSiz);
		uint8_t* o51 = getOption(dhcp->options, 51, opSiz);
		t1 = o54[0];
		t2 = o54[1];
		t3 = o54[2];
		t4 = o54[3];

		t5 = o51[0];
		t6 = o51[1];
		t7 = o51[2];
		t8 = o51[3];
	}

	// Set 0 to unused data
	for (i = 6; i < 16; i++) {
		dhcp->chaddr[i] = 0;
	}
	for (i = 0; i < 192; i++) {
		dhcp->data[i] = 0;
	}

	uint8_t opSize = 0;

	// Add DHCP Message type according to packet_type variable
	uint8_t dhcp_packet_type = packet_type;
	if (dhcp_packet_type == 5 || dhcp_packet_type == 6)
		dhcp_packet_type = 3; // DHCP Renew and Rebind are DHCP Request Packets
	opSize = putOption(dhcp->options, opSize, 53, 1, dhcp_packet_type);

	// Add options parameters list
	opSize = putOption(dhcp->options, opSize, 55, 5, 1, 2, 3, 6, 51);

	dhcp->options[opSize++] = 61;
	dhcp->options[opSize++] = 7;
	dhcp->options[opSize++] = 1;
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		dhcp->chaddr[i] = macAddress[i];
		dhcp->options[opSize++] = macAddress[i];
	}

	// If it is DHCP Request and not in renewing/rebinding state
	if (packet_type == 3) {
		dhcp->options[opSize++] = 50;
		dhcp->options[opSize++] = 4;
		for (i = 0; i < IP_ADD_LENGTH; i++) {
			dhcp->options[opSize++] = dhcp->yiaddr[i];
			tempIpAddress[i] = dhcp->yiaddr[i];
		}

		// Put Lease Time option if this is DHCP Request
		opSize = putOption(dhcp->options, opSize, 51, 4, t5, t6, t7, t8);
//		opSize = putOption(dhcp->options, opSize, 51, 4, 0, 0, 0, 60); // Testing for DHCP Renew

		// Put DHCP Server Identifier option if this is DHCP Request
		opSize = putOption(dhcp->options, opSize, 54, 4, t1, t2, t3, t4);

	}

	// End options
	dhcp->options[opSize++] = 255;

	for (i = 0; i < IP_ADD_LENGTH; i++) {

		// Populate CIAddr field if this is DHCP Rebind/Renew/Release
		if (packet_type > 4) {
			dhcp->ciaddr[i] = ipAddress[i];
		} else {
			dhcp->ciaddr[i] = 0;
		}
		dhcp->yiaddr[i] = 0;
		dhcp->siaddr[i] = 0;
		dhcp->giaddr[i] = 0;
	}

	uint16_t udpSize = 240 + opSize; // calculate dynamic options size

	// adjust lengths
	ip->length = htons(((ip->revSize & 0xF) * 4) + 8 + udpSize);
	// 32-bit sum over ip header
	sum = 0;
	etherSumWords(&ip->revSize, 10);
	etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);
	ip->headerChecksum = getEtherChecksum();
	udp->length = htons(8 + udpSize);
	// 32-bit sum over pseudo-header
	sum = 0;
	etherSumWords(ip->sourceIp, 8);
	tmp16 = ip->protocol;
	sum += (tmp16 & 0xff) << 8;
	etherSumWords(&udp->length, 2);
	// add udp header except crc
	etherSumWords(udp, 6);
	etherSumWords(&udp->data, udpSize);
	udp->check = getEtherChecksum();

	// send packet with size = ether + ip header + udp hdr + udp_size
	etherPutPacket(ether, 14 + ((ip->revSize & 0xF) * 4) + 8 + udpSize);
}

// Determines whether packet is DHCP Offer
bool isDhcpOffer(uint8_t packet[]) {

	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	udpFrame* udp = (udpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	dhcpFrame* dhcp = (dhcpFrame*) &udp->data;

	uint8_t i;
	bool ok = true;

	// Check if MAC address matches
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		ok = dhcp->chaddr[i] == macAddress[i];
	}

	uint16_t opSize = htons(udp->length - 248);
	uint8_t* msgType = getOption(dhcp->options, 53, opSize);

	// Check for Client Hardware Address and DHCP Message Type
	if (ok && dhcp->op == 2 && msgType[0] == 2) {
		return true;
	}
	return false;
}

// Determines whether packet is DHCP ACK
// Returns lease time, if not an ACK, returns 0
uint32_t isDhcpAck(uint8_t packet[]) {

	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	udpFrame* udp = (udpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	dhcpFrame* dhcp = (dhcpFrame*) &udp->data;

	uint16_t opSize = htons(udp->length - 248);
	uint8_t* msgType = getOption(dhcp->options, 53, opSize);

	uint8_t i;
	bool ok = true;
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		ok = dhcp->chaddr[i] == macAddress[i];
		serverMacAddress[i] = ether->sourceAddress[i];
	}

	if (ok && dhcp->op == 2 && msgType[0] == 5) {

		uint8_t* leaseTime = getOption(dhcp->options, 51, opSize);

		uint32_t lease = 0;
		lease |= leaseTime[0] << 24;
		lease |= leaseTime[1] << 16;
		lease |= leaseTime[2] << 8;
		lease |= leaseTime[3];

		uint8_t* sn = getOption(dhcp->options, 1, opSize);
		uint8_t* gw = getOption(dhcp->options, 3, opSize);
		uint8_t* dns = getOption(dhcp->options, 6, opSize);

		uint8_t i = 0;
		for (i = 0; i < IP_ADD_LENGTH; i++) {
			ipAddress[i] = tempIpAddress[i];
			dnsAddress[i] = dns[i];
			ipSubnetMask[i] = sn[i];
			ipGwAddress[i] = gw[i];

			serverIpAddress[i] = ip->sourceIp[i];
		}

		return lease;
	}
	return 0;
}

// Sends Gratuituous ARP
void sendGratiousArp(uint8_t packet[]) {

	etherFrame* ether = (etherFrame*) packet;
	uint8_t i;

	for (i = 0; i < HW_ADD_LENGTH; i++) {
		ether->destAddress[i] = 255;
		ether->sourceAddress[i] = macAddress[i];
	}
	ether->frameType = htons(0x0806);

	arpFrame* arp = (arpFrame*) &ether->data;

	// set op to response
	arp->op = htons(1);
	arp->protocolType = htons(0x800);
	arp->hardwareType = htons(1);
	arp->hardwareSize = 6;
	arp->protocolSize = 4;

	for (i = 0; i < IP_ADD_LENGTH; i++) {
		arp->destIp[i] = ipAddress[i];
		arp->sourceIp[i] = ipAddress[i];
	}
	// send packet
	etherPutPacket(ether, 42);
}

// TODO Implementation remaining
// Check if we get response for Gratuituous ARP
bool isArpResponse(uint8_t packet[]) {

	return false;
}

// TCP Functions
bool etherIsTcp(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	tcpFrame* tcp = (tcpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	bool ok;
	uint16_t tmp16;
	ok = (ip->protocol == 6);
	if (ok) {
		// 32-bit sum over pseudo-header

		uint8_t tcpSize = htons(tcp->headerLength) >> 12;
		sum = 0;
		etherSumWords(ip->sourceIp, 8);

		tmp16 = ip->protocol;
		sum += (tmp16 & 0xff) << 8;

		// Calculate TCP Data size
		tmp16 = htons(ip->length) - 20;
		sum += (tmp16 & 0xff) << 8;

		etherSumWords(tcp, tmp16);
		ok = (getEtherChecksum() == 0);
	}
	return ok;
}

// Checks if the TCP packet was a SYN
bool etherIsTcpSYN(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	tcpFrame* tcp = (tcpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	bool ok = (htons(tcp->headerLength) >> 1) & 1;
	return ok;
}


uint32_t currentIsn = 0;

// Checks if the TCP packet was a SYN
bool etherIsTcpAck(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	tcpFrame* tcp = (tcpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	bool ok = (htons(tcp->headerLength) >> 4) & 1;
	uint32_t tmp32 = htols(tcp->ackNumber);
	return ok && tmp32 == currentIsn;
}

// Checks if the TCP packet was Telnet Data
bool etherIsTelnetData(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	uint16_t tL = htons(ip->length);
	tcpFrame* tcp = (tcpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	uint16_t tmp16 = htons(tcp->headerLength);
	bool ok = ((tmp16 >> 3) & 1) && ((tmp16 >> 4) & 1);
	uint32_t tmp32 = htols(tcp->ackNumber);
	return ok && tmp32 == currentIsn;
}

// Send SYN ACK in response to SYN
void etherSendTcpSynAck(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	tcpFrame* tcp = (tcpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));

	uint8_t i, tmp8;
	// swap source and destination fields
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		tmp8 = ether->destAddress[i];
		ether->destAddress[i] = ether->sourceAddress[i];
		ether->sourceAddress[i] = tmp8;
	}
	for (i = 0; i < IP_ADD_LENGTH; i++) {
		tmp8 = ip->destIp[i];
		ip->destIp[i] = ip->sourceIp[i];
		ip->sourceIp[i] = tmp8;
	}
	// set source port of resp will be dest port of req
	// dest port of resp will be left at source port of req
	// unusual nomenclature, but this allows a different tx
	// and rx port on other machine
	uint16_t tmp16 = tcp->destPort;
	tcp->destPort = tcp->sourcePort;
	tcp->sourcePort = tmp16;

	// Set ACK Number as +1 of Seq Number
	tcp->ackNumber = htols(htols(tcp->sequenceNumber) + 1);

	// Increment Seq Number (since no data is sent)
	tcp->sequenceNumber = htols(currentIsn++);

	tcp->headerLength = htons(tcp->headerLength);

	// Set ACK Flag
	tcp->headerLength |= (1 << 4);

	uint8_t tcpSize = tcp->headerLength >> 12;

	tcp->headerLength = htons(tcp->headerLength);

	// 32-bit sum over ip header
	sum = 0;
	etherSumWords(&ip->revSize, 10);
	etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);
	ip->headerChecksum = getEtherChecksum();

	// Calculate TCP Checksum
	tcp->checksum = 0;
	sum = 0;
	etherSumWords(ip->sourceIp, 8);
	tmp16 = ip->protocol;
	sum += (tmp16 & 0xff) << 8;
	sum += ((tcpSize * 4) & 0xff) << 8;
	etherSumWords(tcp, tcpSize * 4);

	tcp->checksum = getEtherChecksum();

	// send packet with size = ether + ip header + tcp_size
	etherPutPacket(ether, 14 + ((ip->revSize & 0xF) * 4) + (tcpSize * 4));
}

// Send Telnet data
void etherSendTelnetData(uint8_t packet[], uint8_t* telnetData, uint8_t telnetSize) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	tcpFrame* tcp = (tcpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));

	uint8_t i, tmp8;
	// swap source and destination fields
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		tmp8 = ether->destAddress[i];
		ether->destAddress[i] = ether->sourceAddress[i];
		ether->sourceAddress[i] = tmp8;
	}
	for (i = 0; i < IP_ADD_LENGTH; i++) {
		tmp8 = ip->destIp[i];
		ip->destIp[i] = ip->sourceIp[i];
		ip->sourceIp[i] = tmp8;
	}
	// set source port of resp will be dest port of req
	// dest port of resp will be left at source port of req
	// unusual nomenclature, but this allows a different tx
	// and rx port on other machine
	uint16_t tmp16 = tcp->destPort;
	tcp->destPort = tcp->sourcePort;
	tcp->sourcePort = tmp16;

	tcp->ackNumber = htols(htols(tcp->sequenceNumber) + 1);
	tcp->sequenceNumber = htols(currentIsn);
	currentIsn += telnetSize;

	tcp->headerLength = htons(tcp->headerLength);

	uint8_t tcpSize = (tcp->headerLength >> 12);

	tcp->headerLength = htons(tcp->headerLength);

	// Copy data to TCP Frame
	uint8_t* copyData;
	copyData = &tcp->data;
	for (i = 0; i < telnetSize; i++)
		copyData[i] = telnetData[i];

	// Update lenght in IP Header
	ip->length = htons(40 + telnetSize);

	// 32-bit sum over ip header
	sum = 0;
	etherSumWords(&ip->revSize, 10);
	etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);
	ip->headerChecksum = getEtherChecksum();

	// Calculate TCP Checksum
	tcp->checksum = 0;
	sum = 0;
	etherSumWords(ip->sourceIp, 8);
	tmp16 = ip->protocol;
	sum += (tmp16 & 0xff) << 8;
	sum += ((tcpSize * 4 + telnetSize) & 0xff) << 8;
	etherSumWords(tcp, tcpSize * 4);
	etherSumWords(&tcp->data, telnetSize);

	tcp->checksum = getEtherChecksum();

	// send packet with size = ether + udp hdr + ip header + tcp_size + tcp_data
	etherPutPacket(ether, 14 + ((ip->revSize & 0xF) * 4) + (tcpSize * 4) + telnetSize);
}

// Checks if the TCP packet was a FIN_ACK
bool etherIsTcpFINACK(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	tcpFrame* tcp = (tcpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));
	bool ok = (htons(tcp->headerLength) >> 4) & 1 && htons(tcp->headerLength) & 1;
	uint32_t tmp32 = htols(tcp->ackNumber);
	return ok && tmp32 == currentIsn;
}


// Send ACK and FIN ACK in response to FIN ACK
void etherSendAckFinAck(uint8_t packet[]) {
	etherFrame* ether = (etherFrame*) packet;
	ipFrame* ip = (ipFrame*) &ether->data;
	tcpFrame* tcp = (tcpFrame*) ((uint8_t*) ip + ((ip->revSize & 0xF) * 4));

	uint8_t i, tmp8;
	// swap source and destination fields
	for (i = 0; i < HW_ADD_LENGTH; i++) {
		tmp8 = ether->destAddress[i];
		ether->destAddress[i] = ether->sourceAddress[i];
		ether->sourceAddress[i] = tmp8;
	}
	for (i = 0; i < IP_ADD_LENGTH; i++) {
		tmp8 = ip->destIp[i];
		ip->destIp[i] = ip->sourceIp[i];
		ip->sourceIp[i] = tmp8;
	}
	// set source port of resp will be dest port of req
	// dest port of resp will be left at source port of req
	// unusual nomenclature, but this allows a different tx
	// and rx port on other machine
	uint16_t tmp16 = tcp->destPort;
	tcp->destPort = tcp->sourcePort;
	tcp->sourcePort = tmp16;

	// Set ACK Number as +1 of Seq Number
	tcp->ackNumber = htols(htols(tcp->sequenceNumber) + 1);

	// Increment Seq Number (since no data is sent)
	tcp->sequenceNumber = htols(currentIsn++);

	tcp->headerLength = htons(tcp->headerLength);

	// Set ACK Flag
	tcp->headerLength |= (1 << 4);

	// Unset FIN Flag
	tcp->headerLength = tcp->headerLength >> 1;
	tcp->headerLength = tcp->headerLength << 1;


	uint8_t tcpSize = tcp->headerLength >> 12;

	tcp->headerLength = htons(tcp->headerLength);

	// 32-bit sum over ip header
	sum = 0;
	etherSumWords(&ip->revSize, 10);
	etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);
	ip->headerChecksum = getEtherChecksum();


	tcp->checksum = 0;
	sum = 0;
	etherSumWords(ip->sourceIp, 8);
	tmp16 = ip->protocol;
	sum += (tmp16 & 0xff) << 8;
	sum += ((tcpSize * 4) & 0xff) << 8;
	etherSumWords(tcp, tcpSize * 4);

	tcp->checksum = getEtherChecksum();

	// send packet with size = ether + ip header + tcp_size
	etherPutPacket(ether, 14 + ((ip->revSize & 0xF) * 4) + (tcpSize * 4));

	tcp->headerLength = htons(tcp->headerLength);

	// Set FIN Flag
	tcp->headerLength |= 1;

	tcp->headerLength = htons(tcp->headerLength);

	// Recalculate checksum
	tcp->checksum = 0;
	sum = 0;
	etherSumWords(ip->sourceIp, 8);
	tmp16 = ip->protocol;
	sum += (tmp16 & 0xff) << 8;
	sum += ((tcpSize * 4) & 0xff) << 8;
	etherSumWords(tcp, tcpSize * 4);

	tcp->checksum = getEtherChecksum();

	// send packet with size = ether + ip header + tcp_size
	etherPutPacket(ether, 14 + ((ip->revSize & 0xF) * 4) + (tcpSize * 4));

}


// Get an option from DHCP options field. MUST have the option number
uint8_t* getOption(uint8_t options[0], uint8_t number, uint8_t size) {
	uint8_t i = 0;
	while (i < size) {
		if (options[i] == number) {
			// return the start of the value of that option
			return &options[i + 2];
		} else {
			// Increment i by the length of current option
			i += options[i + 1] + 2;
		}
	}
	return NULL;
}

// Put an option
uint8_t putOption(uint8_t options[0], uint8_t size, uint8_t number, uint8_t count, ...) {
	options[size++] = number;
	options[size++] = count;

	uint8_t i = 0;

	va_list ap;
	va_start(ap, count);

	for (i = 0; i < count; i++) {
		options[size++] = va_arg(ap, uint8_t);
	}
	return size;
}
