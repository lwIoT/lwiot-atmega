/*
 * Hardware I2C algorithm definition.
 *
 * @author Michel Megens
 * @email  dev@bietje.net
 */

#include <stdlib.h>
#include <lwiot.h>

#include <lwiot/types.h>
#include <lwiot/log.h>
#include <lwiot/sharedpointer.h>

#include <lwiot/io/i2calgorithm.h>
#include <lwiot/io/gpiopin.h>
#include <lwiot/kernel/lock.h>

#include <lwiot/io/hardwarei2calgorithm.h>
#include <lwiot/avr/hardwarei2calgorithm.h>

#include <util/twi.h>

#include "i2c/avri2c.h"

#define ATMEGA_MAX_MSGS 12

#define CMD_STA 1U
#define CMD_STO 2U
#define CMD_REPSTA 4U
#define CMD_RD 8U
#define CMD_ACK 16U

struct i2c_msg {
	uint8_t *buff;
	uint8_t sla;
	size_t len;
	uint8_t flags;
};

#define cbi(port,bit) \
	(port) &= ~(1 << (bit))

static volatile uint8_t msg_index = 0;
static volatile uint8_t msg_num = 0;
static struct i2c_msg xfer_msgs[ATMEGA_MAX_MSGS];

namespace lwiot
{
	namespace avr
	{
		HardwareI2CAlgorithm::HardwareI2CAlgorithm(const lwiot::GpioPin &scl,
				const lwiot::GpioPin &sda, uint32_t frequency) : lwiot::HardwareI2CAlgorithm(scl, sda, frequency)
		{
			TWCR = _BV(TWEN) | _BV(TWEA);
			this->setFrequency(frequency);
		}

		HardwareI2CAlgorithm::HardwareI2CAlgorithm(HardwareI2CAlgorithm &&other) noexcept :
			lwiot::HardwareI2CAlgorithm(stl::forward<HardwareI2CAlgorithm>(other))
		{
		}

		HardwareI2CAlgorithm::~HardwareI2CAlgorithm()
		{
			cbi(TWCR, TWEA);
			cbi(TWCR, TWEN);
			cbi(TWCR, TWIE);
		}

		HardwareI2CAlgorithm& HardwareI2CAlgorithm::operator=(HardwareI2CAlgorithm &&rhs) noexcept
		{
			this->move(rhs);
			return *this;
		}

		void HardwareI2CAlgorithm::start(uint16_t sla, bool repeated)
		{
			struct i2c_msg *msg;

			msg = &xfer_msgs[msg_index];
			msg->sla = (uint8_t) sla;

			if(repeated)
				msg->flags |= CMD_REPSTA;
			else
				msg->flags |= CMD_STA;

			msg->buff = (uint8_t *) &msg->sla;
			msg->len = sizeof(uint8_t);

			msg_index += 1;
			msg_num += 1;
		}

		void HardwareI2CAlgorithm::stop()
		{
			struct i2c_msg *msg ;

			msg = &xfer_msgs[msg_index - 1];
			msg->flags |= CMD_STO;
		}

		void HardwareI2CAlgorithm::write(const uint8_t *byte, bool ack)
		{
			struct i2c_msg *msg ;

			msg = &xfer_msgs[msg_index];
			msg->buff = (uint8_t*) byte;
			msg->len = sizeof(*byte);

			if(ack)
				msg->flags = CMD_ACK;

			msg_index += 1;
			msg_num += 1;
		}

		void HardwareI2CAlgorithm::write(const uint8_t *bytes, size_t length, bool ack)
		{
			struct i2c_msg *msg ;

			UNUSED(ack);

			msg = &xfer_msgs[msg_index];
			msg->buff = (uint8_t*) bytes;
			msg->len = length;

			if(ack)
				msg->flags |= CMD_ACK;

			msg_index += 1;
			msg_num += 1;
		}

		void HardwareI2CAlgorithm::read(uint8_t *byte, bool ack)
		{

			struct i2c_msg *msg ;

			msg = &xfer_msgs[msg_index];
			msg->buff = byte;
			msg->len = sizeof(*byte);

			if(ack)
				msg->flags |= CMD_ACK;

			msg->flags |= CMD_RD;

			msg_index += 1;
			msg_num += 1;
		}

		void HardwareI2CAlgorithm::read(uint8_t *bytes, size_t length, bool ack)
		{
			struct i2c_msg *msg ;

			msg = &xfer_msgs[msg_index];
			msg->buff = bytes;
			msg->len = length;

			if(ack)
				msg->flags |= CMD_ACK;

			msg->flags |= CMD_RD;

			msg_index += 1;
			msg_num += 1;
		}

		int HardwareI2CAlgorithm::flush() const
		{
			struct i2c_msg *msg;

			msg_index = 0;

			for(int idx = 0; idx < msg_num; idx++) {
				msg = &xfer_msgs[idx];

				if(msg->flags & CMD_STA || msg->flags & CMD_REPSTA) {
					i2c_write_start(msg->buff[0]);
					continue;
				}

				if((msg->flags & CMD_RD) != 0) {
					for(int j = 0; j < msg->len; j++) {
						msg->buff[j] = i2c_read((msg->flags & CMD_ACK) != 0);
					}
				} else {
					for(int j = 0; j < msg->len; j++) {
						uint8_t ack = !i2c_write(msg->buff[j]);
						uint8_t expected = (msg->flags & CMD_ACK) != 0;

						expected = expected && (j+1) < msg->len;

						if(expected && !ack) {
							print_dbg("ACK not received!\n");
							return -EINVALID;
						}
					}
				}

				if(msg->flags & CMD_STO) {
					i2c_write_stop();
				}
			}

			return -EOK;
		}

		void HardwareI2CAlgorithm::reset()
		{
			memset(xfer_msgs, 0, sizeof(xfer_msgs));
			msg_index = 0;
			msg_num = 0;
		}

		void HardwareI2CAlgorithm::setFrequency(const uint32_t &freq)
		{
			cbi(TWSR, TWPS0);
			cbi(TWSR, TWPS1);
			TWBR = (uint8_t ) (((F_CPU / freq) - 16) / 2);
		}
	}
}
