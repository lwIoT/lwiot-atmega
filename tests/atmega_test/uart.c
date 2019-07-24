/*
 * UART configuration.
 *
 * @author Michel Megens
 * @email  dev@bietje.net
 */

#include <stdlib.h>
#include <stdio.h>

#include <avr/io.h>

#define UART_BAUD_RATE  9600
#define UART_BAUD_REGISTERS  (((F_CPU / (UART_BAUD_RATE * 16UL))) - 1)

int printCHAR(char character, FILE *stream)
{
	if(character == '\n')
		printCHAR('\r', stream);

	while ((UCSR0A & (1 << UDRE0)) == 0) {};
	UDR0 = character;
	return 0;
}

static int getCHAR(FILE *stream)
{
	int c;

	while(!(UCSR0A & _BV(RXC0)));
	c = UDR0;

	return c;
}

static FILE uart_str = FDEV_SETUP_STREAM(printCHAR, getCHAR, _FDEV_SETUP_RW);

void uart_init(void)
{
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);
	UBRR0L = UART_BAUD_REGISTERS;

	stdout = stdin = stderr = &uart_str;
}

