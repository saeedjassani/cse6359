// Ethernet Example
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

// Pinning for IoT projects with wireless modules:
// N24L01+ RF transceiver
//   MOSI (SSI0Tx) on PA5
//   MISO (SSI0Rx) on PA4
//   SCLK (SSI0Clk) on PA2
//   ~CS on PE0
//   INT on PB2
// Xbee module
//   DIN (UART1TX) on PC5
//   DOUT (UART1RX) on PC4

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "eth0.h"
#include "gpio.h"
#include "spi0.h"
#include "uart0.h"
#include "wait.h"
#include "eprom.h"
#include "timer.h"

// Pins
#define RED_LED PORTF,1
#define BLUE_LED PORTF,2
#define GREEN_LED PORTF,3
#define PUSH_BUTTON PORTF,4

// DHCP States (and static [DHCP Disabled] state)
#define STATIC 0
#define INIT 1
#define SELECTING 2
#define REQUESTING 3
#define BOUND 4
#define RENEWING 5
#define REBINDING 6

// TCP States
#define LISTEN 0
#define SYN_RECEIVED 1
#define ESTABLISHED 2
#define FINWAIT_1 3
#define FINWAIT_2 4
#define TIME_WAIT 5
#define CLOSED 6

//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw() {
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV
	        | (4 << SYSCTL_RCC_SYSDIV_S);

	// Enable clocks
	enablePort(PORTF);
	_delay_cycles(3);

	// Configure LED and pushbutton pins
	selectPinPushPullOutput(RED_LED);
	selectPinPushPullOutput(GREEN_LED);
	selectPinPushPullOutput(BLUE_LED);
	selectPinDigitalInput(PUSH_BUTTON);

	initEeprom();
}

void displayConnectionInfo() {
	uint8_t i;
	char str[10];
	uint8_t mac[6];
	uint8_t ip[4];
	etherGetMacAddress(mac);
	putsUart0("HW: ");
	for (i = 0; i < 6; i++) {
		sprintf(str, "%02x", mac[i]);
		putsUart0(str);
		if (i < 6 - 1)
			putcUart0(':');
	}
	putsUart0("\r\n");
	etherGetIpAddress(ip);
	putsUart0("IP: ");
	for (i = 0; i < 4; i++) {
		sprintf(str, "%u", ip[i]);
		putsUart0(str);
		if (i < 4 - 1)
			putcUart0('.');
	}
	if (etherIsDhcpEnabled())
		putsUart0(" (dhcp)");
	else putsUart0(" (static)");
	putsUart0("\r\n");
	etherGetIpSubnetMask(ip);
	putsUart0("SN: ");
	for (i = 0; i < 4; i++) {
		sprintf(str, "%u", ip[i]);
		putsUart0(str);
		if (i < 4 - 1)
			putcUart0('.');
	}
	putsUart0("\r\n");
	etherGetIpGatewayAddress(ip);
	putsUart0("GW: ");
	for (i = 0; i < 4; i++) {
		sprintf(str, "%u", ip[i]);
		putsUart0(str);
		if (i < 4 - 1)
			putcUart0('.');
	}
	putsUart0("\r\n");
	etherGetDNSAddress(ip);
	putsUart0("DNS: ");
	for (i = 0; i < 4; i++) {
		sprintf(str, "%u", ip[i]);
		putsUart0(str);
		if (i < 4 - 1)
			putcUart0('.');
	}
	putsUart0("\r\n");
	if (etherIsLinkUp())
		putsUart0("Link is up\r\n");
	else putsUart0("Link is down\r\n");
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

// Max packet is calculated as:
// Ether frame header (18) + Max MTU (1500) + CRC (4)
#define MAX_PACKET_SIZE 1522

bool sendDhcp = false, renewLease = false, sendRenewRequest = false, sendRebindRequest = false,
        rebind = false, leaseEnd = false, safeToUseIp = false, dhcpRelease = false, transitionToInit = false;

void sendDhcpDiscovery() {
	sendDhcp = true;
}

void startT1Timer() {
	renewLease = true;
}

void startRenewing() {
	sendRenewRequest = true;
}

void startT2Timer() {
	rebind = true;
}

void startRebinding() {
	sendRebindRequest = true;
}

void leaseEndTimer() {
	leaseEnd = true;
}

void arpResponse() {
	safeToUseIp = true;
}

void startDeclineTimer() {
	transitionToInit = true;
}

int main(void) {
	uint8_t* udpData;
	uint8_t data[MAX_PACKET_SIZE];
	uint8_t state = 0, tcp_state = 0;

	// Init controller
	initHw();

	// Setup UART0
	initUart0();
	setUart0BaudRate(115200, 40e6);

	// Init Timer
	initTimer();
	bool dhcpMode = false;

	// Init ethernet interface (eth0) and Get DHCP mode from EEPROM
	putsUart0("\r\nStarting eth0\r\n");
	etherInit(ETHER_UNICAST | ETHER_BROADCAST | ETHER_HALFDUPLEX);
	etherSetMacAddress(2, 3, 4, 5, 6, 136);

	dhcpMode = etherIsDhcpEnabled();
	if (dhcpMode) {
		state = INIT;
	} else {
		state = STATIC;
	}

	waitMicrosecond(100000);
	displayConnectionInfo();

	// Flash LED
	setPinValue(GREEN_LED, 1);
	waitMicrosecond(100000);
	setPinValue(GREEN_LED, 0);
	waitMicrosecond(100000);

	// Main Loop
	// RTOS and interrupts would greatly improve this code,
	// but the goal here is simplicity
	while (true) {
		// Put terminal processing here
		if (kbhitUart0()) {
			USER_DATA info;
			// Get the string from the user
			getsUart0(&info);
			// Parse fields
			parseFields(&info);

			// Echo back the parsed field information (type and fields)
//			uint8_t i;
			putsUart0("\r\n");
//			for (i = 0; i < info.fieldCount; i++) {
//				putcUart0(info.fieldType[i]);
//				putcUart0('\t');
//				putsUart0(&info.buffer[info.fieldPosition[i]]);
//				putsUart0("\r\n");
//			}

			bool valid = false;

			// set IP | GW | DNS | SN w.x.y.z
			if (isCommand(&info, "set", 5)) {
				char* add = getFieldString(&info, 2);
				valid = true;

				if (state != STATIC) {
					putsUart0("DHCP mode is on. ");
					valid = false;
				} else {
					if (mystrcmp("ip", add)) {
						etherSetIpAddress(getFieldInteger(&info, 3), getFieldInteger(&info, 4),
						                  getFieldInteger(&info, 5), getFieldInteger(&info, 6));
					} else if (mystrcmp("gw", add)) {
						etherSetIpGatewayAddress(getFieldInteger(&info, 3), getFieldInteger(&info, 4),
						                         getFieldInteger(&info, 5), getFieldInteger(&info, 6));
					} else if (mystrcmp("dns", add)) {
						etherSetDNSAddress(getFieldInteger(&info, 3), getFieldInteger(&info, 4),
						                   getFieldInteger(&info, 5), getFieldInteger(&info, 6));
					} else if (mystrcmp("sn", add)) {
						etherSetIpSubnetMask(getFieldInteger(&info, 3), getFieldInteger(&info, 4),
						                     getFieldInteger(&info, 5), getFieldInteger(&info, 6));
					} else {
						valid = false;
					}
				}
			}

			// dhcp ON | OFF | REFRESH | RELEASE
			if (isCommand(&info, "dhcp", 1)) {
				char* str = getFieldString(&info, 2);
				valid = true;
				if (mystrcmp(str, "on")) {
					etherEnableDhcpMode();
					state = INIT;
				} else if (mystrcmp(str, "off")) {
					etherDisableDhcpMode();
					state = STATIC;
					getDetailsFromEprom();

					// Stop timers
					stopAllTimers();
				} else if (mystrcmp(str, "release")) {

					if (state == STATIC) {
						putsUart0("DHCP mode is off ");
						valid = false;
					} else {
						dhcpRelease = true;
					}

				} else if (mystrcmp(str, "refresh")) {

					if (state == STATIC) {
						putsUart0("DHCP mode is off ");
						valid = false;
					} else {
						sendRenewRequest = true;
					}

				} else {
					valid = false;
				}
			}

			// ifconfig
			if (isCommand(&info, "ifconfig", 0)) {
				displayConnectionInfo();
				valid = true;
			}

			// reboot
			if (isCommand(&info, "reboot", 0)) {
				NVIC_APINT_R |= NVIC_APINT_SYSRESETREQ;
				valid = true;
			}

			if (!valid)
				putsUart0("Invalid command\n");
		}

		if (state == INIT) {
			// Send Discover once and start periodic timer
			sendDhcp = true;
			startPeriodicTimer(sendDhcpDiscovery, 15);
		}

		if (sendDhcp) {
			sendDhcp = false;
			etherSendDhcpPacket(data, 1);
			state = SELECTING;
		}

		// After waiting 2 seconds after sending gratuituous ARP, transition to BOUND state
		if (safeToUseIp && state == REQUESTING) {
			state = BOUND;
			setPinValue(GREEN_LED, 1);
			waitMicrosecond(100000);
			setPinValue(GREEN_LED, 0);
//			putsUart0("\r\nGot dynamic IP Address\r\n");
//			displayConnectionInfo();
		}

		if (renewLease) {
			state = RENEWING;
			renewLease = false;
			startPeriodicTimer(startRenewing, 15);
		}

		if (sendRenewRequest) {
			etherSendDhcpPacket(data, 4);
			sendRenewRequest = false;
		}

		if (rebind) {
			stopTimer(startRenewing);
			state = REBINDING;
			rebind = false;
			startPeriodicTimer(startRebinding, 15);
		}

		if (sendRebindRequest) {
			sendRebindRequest = false;
			etherSendDhcpPacket(data, 5);
		}

		// Transition to INIT state once lease ends (without renewing) and set IP to 0.0.0.0 (so that we don't use it)
		if (leaseEnd) {
			stopTimer(startRebinding);
			state = INIT;
			leaseEnd = false;
			etherSetIpAddressToZeroes();
		}

		// Send DHCP Release and transition to STATIC state
		if (dhcpRelease) {
			dhcpRelease = false;

			etherSendDhcpPacket(data, 7);

			etherDisableDhcpMode();
			state = STATIC;
			getDetailsFromEprom();

			stopAllTimers();

		}

		// Transition to INIT state after sending DHCP Decline and waiting 10 secs (as in RFC)
		if (transitionToInit) {
			transitionToInit = false;
			state = INIT;
		}

		// Packet processing
		if (etherIsDataAvailable()) {
			if (etherIsOverflow()) {
				setPinValue(RED_LED, 1);
				waitMicrosecond(100000);
				setPinValue(RED_LED, 0);
			}

			// Get packet
			etherGetPacket(data, MAX_PACKET_SIZE);

			// Handle ARP request
			if (etherIsArpRequest(data)) {
				etherSendArpResponse(data);
			}

			// Handle IP datagram
			if (etherIsIp(data)) {
				if (etherIsIpUnicast(data)) {
					// handle icmp ping request
					if (etherIsPingRequest(data)) {
						etherSendPingResponse(data);
					}

					// Process UDP datagram
					// test this with a udp send utility like sendip
					//   if sender IP (-is) is 192.168.1.198, this will attempt to
					//   send the udp datagram (-d) to 192.168.1.199, port 1024 (-ud)
					// sudo sendip -p ipv4 -is 192.168.1.198 -p udp -ud 1024 -d "on" 192.168.1.199
					// sudo sendip -p ipv4 -is 192.168.1.198 -p udp -ud 1024 -d "off" 192.168.1.199
					if (etherIsUdp(data)) {

						udpData = etherGetUdpData(data);
						if (strcmp((char*) udpData, "on") == 0)
							setPinValue(GREEN_LED, 1);
						if (strcmp((char*) udpData, "off") == 0)
							setPinValue(GREEN_LED, 0);
						etherSendUdpResponse(data, (uint8_t*) "Received", 9);
					}

					// Handle ARP Response to Gratuituous ARP
					if (isArpResponse(data)) { // Implementation remaining (Didn't had sample ARP response packet)
						// Blink RED LED
						setPinValue(RED_LED, 1);
						waitMicrosecond(100000);
						setPinValue(RED_LED, 0);

						// Send decline message to server
						etherSendDhcpPacket(data, 4);

						// Stop all Timers and start a oneshot timer to transition to INIT state after 10 seconds
						stopAllTimers();
						startOneshotTimer(startDeclineTimer, 10);

					}


					// Handle TCP Packets
					if (etherIsTcp(data)) {

						// Handle TCP SYN packets
						if (etherIsTcpSYN(data)) {
							// Send SYN ACK in response to SYN
							etherSendTcpSynAck(data);
							tcp_state = SYN_RECEIVED;
						} else if (etherIsTcpAck(data) && tcp_state == SYN_RECEIVED) { // Handle TCP ACK

							// Transition to
							tcp_state = ESTABLISHED;
						} else if (etherIsTelnetData(data)) { // Handle TelnetDate packet

							// Send Telnet Data
							etherSendTelnetData(data, (uint8_t*) "Hello", 5);
						} else if (etherIsTcpFINACK(data)) {
							setPinValue(BLUE_LED, 1);
							waitMicrosecond(100000);
							setPinValue(BLUE_LED, 0);
							etherSendAckFinAck(data);
							tcp_state = FINWAIT_1;

						} else if (etherIsTcpAck(data) && tcp_state == SYN_RECEIVED) { // Handle TCP ACK
							tcp_state = CLOSED;

						}
					}
				}

				// Handle IP Broadcast packets
				if (etherIsIpBroadcast(data)) {
					if (etherIsUdp(data)) {

						// If DHCP Offer and state is SELECTING, send DHCP Request and transition to REQUESTING state
						if (isDhcpOffer(data) && state == SELECTING) {
							etherSendDhcpPacket(data, 3);
							state = REQUESTING;
						}

						// If DHCP ACK, get the lease time and start the timers
						uint32_t lease = isDhcpAck(data);
						if (lease > 0 && (state == REQUESTING || state == RENEWING || state == REBINDING)) {
							stopAllTimers();

							if (state == REQUESTING)
								sendGratiousArp(data); // Send Gratuitous ARP only if getting IP for the first timer (not renew/rebind)
							// Start T1 and T2 one shot timers which will start periodic timers
							startOneshotTimer(startT1Timer, lease * 0.5);
							startOneshotTimer(startT2Timer, lease * 0.875);
							startOneshotTimer(leaseEndTimer, lease);
							startOneshotTimer(arpResponse, 2);

						}

					}

				}

			}
		}
	}
}

