/*
 * AVR I2C implementation.
 * 
 * @author Michel Megens
 * @email  dev@bietje.net
 */

#include "avri2c.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <lwiot.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/twi.h>

#define F_SCL 100000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)
#define I2C_READ 0x1U
#define I2C_WRITE 0x0U

#define TMO 200

void avr_i2c_init(void)
{
	TWBR = TWBR_val;
}

static void i2c_reset_bus()
{
	TWCR = 0;
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
}

uint8_t i2c_write_start(uint8_t address)
{
	time_t tmo = lwiot_tick_ms() + TMO;

	i2c_reset_bus();
	while( !(TWCR & (1<<TWINT)) ) {
		if(tmo < lwiot_tick_ms()) {
			i2c_reset_bus();
			return 1;
		}
	}
	
	if((TWSR & 0xF8) != TW_START)
		return 1;
	
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	while( !(TWCR & (1<<TWINT)) ) {
		if(tmo < lwiot_tick_ms()) {
			i2c_reset_bus();
			return 1;
		}
	}
	
	uint8_t twst = TW_STATUS & 0xF8;
	if((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) )
		return 1;
	
	return 0;
}

uint8_t i2c_write(uint8_t data)
{
	time_t tmo = lwiot_tick_ms() + TMO;

	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while( !(TWCR & (1<<TWINT)) ) {
		if(tmo < lwiot_tick_ms()) {
			i2c_reset_bus();
			return 1;
		}

	}
	
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK )
		return 1;
	
	return 0;
}

uint8_t i2c_read_ack(void)
{
	time_t tmo = lwiot_tick_ms() + TMO;

	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while( !(TWCR & (1<<TWINT)) ) {
		if(tmo < lwiot_tick_ms()) {
			i2c_reset_bus();
			return 1;
		}
	}

	return TWDR;
}

uint8_t i2c_read_nack(void)
{
	time_t tmo = lwiot_tick_ms() + TMO;

	TWCR = (1<<TWINT) | (1<<TWEN);
	while( !(TWCR & (1<<TWINT)) ) {
		if(tmo < lwiot_tick_ms()) {
			i2c_reset_bus();
			return 1;
		}
	}

	return TWDR;
}

uint8_t i2c_read(bool ack)
{
	uint8_t byte;

	if(ack)
		byte = i2c_read_ack();
	else
		byte = i2c_read_nack();

	return byte;
}

uint8_t i2c_transmit(uint8_t address, uint8_t* data, uint16_t length, uint8_t stop)
{
	if (i2c_write_start(address | I2C_WRITE))
		return 1;
	
	for (uint16_t i = 0; i < length; i++) {
		if (i2c_write(data[i])) return 1;
	}

	if(stop)
		i2c_write_stop();

	return 0;
}

uint8_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length, uint8_t stop)
{
	if (i2c_write_start(address | I2C_READ))
		return 1;
	
	for (uint16_t i = 0; i < (length-1); i++) {
		data[i] = i2c_read_ack();
	}

	data[(length-1)] = i2c_read_nack();

	if(stop)
		i2c_write_stop();
	
	return 0;
}

uint8_t i2c_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
	if (i2c_write_start(devaddr | 0x00))
		return 1;

	i2c_write(regaddr);

	for (uint16_t i = 0; i < length; i++) {
		if (i2c_write(data[i]))
			return 1;
	}

	i2c_write_stop();
	return 0;
}

uint8_t i2c_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
	if (i2c_write_start(devaddr))
		return 1;

	i2c_write(regaddr);

	if (i2c_write_start(devaddr | 0x01))
		return 1;

	for (uint16_t i = 0; i < (length-1); i++) {
		data[i] = i2c_read_ack();
	}

	data[(length-1)] = i2c_read_nack();
	i2c_write_stop();

	return 0;
}

void i2c_write_stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}
