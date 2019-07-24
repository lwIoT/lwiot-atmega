/*
 * AVR FreeRTOS unit test.
 */

#include <stdlib.h>
#include <stdio.h>
#include <lwiot.h>

#include <lwiot/log.h>
#include <lwiot/types.h>
#include <lwiot/functor.h>
#include <lwiot/system.h>

#include <lwiot/io/gpiochip.h>
#include <lwiot/io/gpiopin.h>
#include <lwiot/io/watchdog.h>
#include <lwiot/io/i2cbus.h>
#include <lwiot/io/hardwarei2calgorithm.h>

#include <lwiot/avr/hardwarei2calgorithm.h>
#include <lwiot/device/apds9301sensor.h>

#include <lwiot/util/application.h>
#include <lwiot/util/datetime.h>

#include <util/delay.h>
#include <avr/io.h>

extern "C" void uart_init(void);

class AtmegaApp : public lwiot::Functor {
protected:
	void run() override
	{
		bool status = false;
		uint8_t portd = PORTD;
		lwiot::GpioPin led(13);

		lwiot::GpioPin scl(19), sda(18);
		lwiot::HardwareI2CAlgorithm *algo = new lwiot::avr::HardwareI2CAlgorithm(scl, sda, 400000);
		lwiot::I2CBus bus(algo);
		lwiot::Apds9301Sensor apds(bus);

		led.output();
		wdt.enable(3000);
		apds.begin();

		while(true) {
			double lux = 0.0;

			led.write(status);
			status = !status;
			wdt.reset();
			apds.getLux(lux);

			print_dbg("PING! Lux: %i\n", static_cast<int>(lux));
			lwiot::System::delay(1200);
		}
	}

};

int main(void)
{
	uart_init();
	AtmegaApp lwiot;
	lwiot::Application app(lwiot);

	app.start();

	return -1;
}

