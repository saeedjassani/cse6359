// UART0 Library
// Jason Losh
// Modified by Saeed Jassani

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0() {
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV
	        | (4 << SYSCTL_RCC_SYSDIV_S);

	// Set GPIO ports to use APB (not needed since default configuration -- for clarity)
	SYSCTL_GPIOHBCTL_R = 0;

	// Enable clocks
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
	_delay_cycles(3);

	// Configure UART0 pins
	GPIO_PORTA_DIR_R |= UART_TX_MASK;                   // enable output on UART0 TX pin
	GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
	GPIO_PORTA_DR2R_R |= UART_TX_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
	GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
	GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
	GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
	// select UART0 to drive pins PA0 and PA1: default, added for clarity

	// Configure UART0 to 115200 baud, 8N1 format
	UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
	UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
	UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
	UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
	UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
	// enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc) {
	uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
	                                                    // where r = fcyc / 16 * baudRate
	UART0_IBRD_R = divisorTimes128 >> 7;                 // set integer value to floor(r)
	UART0_FBRD_R = ((divisorTimes128 + 1)) >> 1 & 63;    // set fractional value to round(fract(r)*64)
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c) {
	while (UART0_FR_R & UART_FR_TXFF)
		;               // wait if uart0 tx fifo full
	UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str) {
	uint8_t i = 0;
	while (str[i] != '\0')
		putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0() {
	while (UART0_FR_R & UART_FR_RXFE)
		;               // wait if uart0 rx fifo empty
	return UART0_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart0() {
	return !(UART0_FR_R & UART_FR_RXFE);
}

void getsUart0(USER_DATA* data) {

	data->charCount = 0;
	char c;
	while ((c = getcUart0())) {

		// Handle Backspace
		if (c == 127) {
			if (data->charCount > 0) {
				data->charCount--;
				putsUart0("<bs>"); // Display <bs> when backspace is pressed on keyboard
			}
			continue;
		}

		// Handle new line and carriage return
		if (c == 10 || c == 13) {
			data->buffer[data->charCount] = '\0';
			putsUart0("\r\n");
			return;
		}

		// Save into buffer if alphabets
		if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {
			if (c >= 'A' && c <= 'Z')
				c += 32; // Converting uppercase letter to lowercase
			data->buffer[data->charCount++] = c;
			putcUart0(c);
			if (data->charCount == MAX_CHARS) {
				data->buffer[data->charCount] = '\0';
				putsUart0("\r\n");
				return;
			}
		} else {
			putcUart0(' '); // All delimeters are displayed as space and are stored as null terminator
			data->buffer[data->charCount++] = '\0';
		}
	}
}

void parseFields(USER_DATA* data) {

	data->fieldCount = 0;
	int i;
	for (i = 0; i < data->charCount;) {
		if (data->buffer[i] != '\0') {
			data->fieldPosition[data->fieldCount] = i;

			// Store appropriate field type based on FIRST character of the current "string part"
			if (data->buffer[i] >= '0' && data->buffer[i] <= '9') {
				data->fieldType[data->fieldCount++] = 'n';
			} else {
				data->fieldType[data->fieldCount++] = 'a';
			}
			while (data->buffer[i++] != '\0')
				;
		} else {
			i++;
		}
	}
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber) {
	if (fieldNumber <= data->fieldCount) {
		return &data->buffer[data->fieldPosition[fieldNumber - 1]];
	} else {
		return 0;
	}
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber) {
	if (fieldNumber <= data->fieldCount && data->fieldType[fieldNumber - 1] == 'n') {
		return atoi(&data->buffer[data->fieldPosition[fieldNumber - 1]]);
	} else {
		return 0;
	}
}

int32_t atoi(char* str) {
	int res = 0, i; // Initialize result

	// Iterate through all characters of input string and
	// update result
	for (i = 0; str[i] != '\0'; ++i)
		res = res * 10 + str[i] - '0';

	return res;
}

// Reference http://www.geeksforgeeks.org/
bool mystrcmp(const char* a, const char* b) {
	while (*a) {
		if (*a != *b) {
			return false;
		}
		a++;
		b++;
	}
	return true;
}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments) {
	return mystrcmp(&data->buffer[data->fieldPosition[0]], strCommand) && minArguments >= data->fieldCount - 1;
}

void reverse(char str[], int length) {
	uint8_t start = 0;
	uint8_t end = length - 1;
	while (start < end) {
		char tmp = *(str + start);
		*(str + start) = *(str + end);
		*(str + end) = tmp;
		start++;
		end--;
	}
}

void parseInt(int32_t num, char* str) {
	uint8_t i = 0;

	/* Handle 0 explicitely, otherwise empty string is printed for 0 */
	if (num == 0) {
		str[i++] = '0';
		str[i] = '\0';
		return;
	}

	// In standard itoa(), negative numbers are handled only with
	// base 10. Otherwise numbers are considered unsigned.

	// Process individual digits
	while (num != 0) {
		int rem = num % 10;
		str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
		num = num / 10;
	}

	str[i] = '\0'; // Append string terminator

	// Reverse the string
	reverse(str, i);

	return;
}
