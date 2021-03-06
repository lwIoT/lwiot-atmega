/*
 * AVR WDT class definition.
 * 
 * @author Michel Megens
 * @email  dev@bietje.net
 */

#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <lwiot.h>

#include <lwiot/log.h>
#include <lwiot/io/watchdog.h>

namespace lwiot { namespace avr
{
	class Watchdog : public lwiot::Watchdog {
	public:
		static Watchdog& instance()
		{
			static Watchdog _instance;
			return _instance;
		}

		virtual ~Watchdog() = default;
		Watchdog(const Watchdog&) = delete;
		const Watchdog& operator =(const Watchdog&) = delete;

		virtual bool enable(uint32_t tmo = 2000) override;
		virtual bool disable() override;
		virtual void reset() override;

	private:
		/* Methods */
		explicit Watchdog();
		friend void wdt_trigger_handler();
	};
}
}
