/*
 * AVR FreeRTOS unit test.
 */

#include <stdlib.h>
#include <stdio.h>
#include <lwiot.h>

#include <lwiot/log.h>
#include <lwiot/types.h>

#include <lwiot/io/gpiochip.h>
#include <lwiot/io/gpiopin.h>
#include <lwiot/io/watchdog.h>
#include <lwiot/io/i2cbus.h>
#include <lwiot/io/gpiopin.h>
#include <lwiot/io/hardwarei2calgorithm.h>
#include <lwiot/avr/hardwarei2calgorithm.h>

#include <lwiot/util/datetime.h>


#include <util/delay.h>
#include <avr/io.h>

extern "C" void uart_init(void);

static void testRead(lwiot::I2CBus& bus)
{
	lwiot::I2CMessage wr(1), rd(3);
	lwiot::stl::Vector<lwiot::I2CMessage*> msgs;

	wr.setAddress(0x6B, false, false);
	wr.write(1);
	wr.setRepeatedStart(true);

	rd.setAddress(0x6B, false, true);
	rd.setRepeatedStart( false);

	msgs.pushback(&wr);
	msgs.pushback(&rd);

	if(bus.transfer(msgs)) {
		print_dbg("Read test successfull!\n");
	} else {
		print_dbg("Read test failed!\n");
	}

	auto& msg = *msgs[1];
	print_dbg("Read data:\n");
	print_dbg("\t1: %u\n", msg[0]);
	print_dbg("\t2: %u\n", msg[1]);
	print_dbg("\t3: %u\n", msg[2]);

	lwiot_sleep(500);
}

static void testSingle(lwiot::I2CBus& bus)
{
	lwiot::I2CMessage wr(1);

	wr.setAddress(0x6B, false, false);
	wr.write(1);
	wr.setRepeatedStart(false);

	if(bus.transfer(wr)) {
		print_dbg("Single test successfull!\n");
	} else {
		print_dbg("Single test failed!\n");
	}

	lwiot_sleep(500);
}

void run(void)
{
	bool status = false;
	uint8_t portd = PORTD;
	lwiot::GpioPin led(13);
	
	lwiot::GpioPin scl(19), sda(18);
	lwiot::HardwareI2CAlgorithm *algo = new lwiot::avr::HardwareI2CAlgorithm(scl, sda, 400000);
	lwiot::I2CBus bus(algo);

	led.output();
	wdt.enable(3000);

	while(true) {
		led.write(status);
		status = !status;
		wdt.reset();
		testSingle(bus);
		testRead(bus);
	}
}

int main(void)
{
	uart_init();
	lwiot_init();

	//printf("AVR lwIoT test started..\n");
	run();

	return -1;
}

