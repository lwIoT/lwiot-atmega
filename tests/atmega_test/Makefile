
AVRUPLOAD=scripts/avr_upload
OBJCOPY=/usr/bin/avr-objcopy
CPUFREQ=16000000

MCU=atmega328p
PROGRAMMER=arduino
BAUD=115200
PORT=/dev/ttyACM0

CC  = avr-gcc
CXX = avr-g++
GCC_FLAGS = -mmcu=atmega328p -DF_CPU=16000000 -Os -std=c11
GXX_FLAGS = -fno-rtti -mmcu=atmega328p -DF_CPU=16000000 -Os -std=c++14

LWIOT_DIR = $(PWD)/../..
APP_HEADERS = . 
LWIOT_HEADERS = -I$(LWIOT_DIR)/lwiot-core/include -I$(LWIOT_DIR)/build/atmega328 \
		-I$(LWIOT_DIR)/atmega/include \
		-I$(LWIOT_DIR)/lwiot-core/source/lib/time

RTOS_OBJ = $(RTOS_SOURCES:.c=.o)

all: build-dir main.o uart.o
	@$(CC) -o build/app.bin -mmcu=atmega328p build/*.o -L$(LWIOT_DIR)/build/atmega328/atmega \
	-L$(LWIOT_DIR)/build/atmega328/lwiot-core/source  -llwiot-platform -llwiot -llwiot-platform -llwiot
	@echo "LINK build/app.bin"

build-dir:
	@mkdir -p `pwd`/build

flash:
	@avr-objcopy -R .eeprom -O ihex build/app.bin build/app.hex
	@$(AVRUPLOAD) -fH $(PWD)/build/app.hex -m $(MCU) -p $(PROGRAMMER) -P $(PORT) \
			-b $(BAUD) -c /etc/avrdude.conf

%.o: %.c
	@echo "CC $(notdir $<) --> $(notdir $@)"
	@$(CC) $(GCC_FLAGS) -I$(RTOS_HEADERS) -I$(APP_HEADERS) $(LWIOT_HEADERS) \
	       	-c -o build/$(notdir $@) $<
%.o: %.S
	@echo "AS $(notdir $<) --> $(notdir $@)"
	@avr-gcc -c -o build/$(notdir $@) -mmcu=atmega328p $<

%.o: %.cpp
	@echo "CC $(notdir $<) --> $(notdir $@)"
	@$(CXX) $(GXX_FLAGS) -I$(RTOS_HEADERS) -I$(APP_HEADERS) $(LWIOT_HEADERS) \
		-c -o build/$(notdir $@) $<

