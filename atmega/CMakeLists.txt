#
# AVR platform build file.
#

SET(RTOS_DIR ${PROJECT_SOURCE_DIR}/external/avr-freertos)

if(NOT CONFIG_STANDALONE)
SET(RTOS_SOURCES
	${RTOS_DIR}/croutine.c
	${RTOS_DIR}/event_groups.c
	${RTOS_DIR}/MemMang/heap_3.c
	${RTOS_DIR}/list.c
	${RTOS_DIR}/portable/port.c
	${RTOS_DIR}/queue.c
	${RTOS_DIR}/stream_buffer.c
	${RTOS_DIR}/tasks.c
	${RTOS_DIR}/timers.c
	${RTOS_DIR}/hooks.c
)
endif()

set(AVR_GENERIC_SOURCES
	${RTOS_SOURCES}
	${PROJECT_SOURCE_DIR}/atmega/system_tick.S
	${PROJECT_SOURCE_DIR}/atmega/gpio/avr.c
	${PROJECT_SOURCE_DIR}/atmega/avrspi.c
	${PROJECT_SOURCE_DIR}/atmega/i2c/avr-i2c.c
	${PROJECT_SOURCE_DIR}/atmega/i2c/i2c.c
	${PROJECT_SOURCE_DIR}/atmega/i2c/avri2calgorithm.cpp

	${PROJECT_SOURCE_DIR}/atmega/avradcchip.cpp
	${PROJECT_SOURCE_DIR}/atmega/system_tick.S
	${PROJECT_SOURCE_DIR}/atmega/avrspibus.cpp
	${PROJECT_SOURCE_DIR}/atmega/avruart.cpp
	${PROJECT_SOURCE_DIR}/atmega/avrwatchdog.cpp
	${PROJECT_SOURCE_DIR}/atmega/gpio/avrgpiochip.cpp

	${PROJECT_SOURCE_DIR}/atmega/soc/avrsystem.cpp
	${PROJECT_SOURCE_DIR}/atmega/soc/cplusplus.cpp
	${PROJECT_SOURCE_DIR}/atmega/soc/critical.c
	${PROJECT_SOURCE_DIR}/atmega/soc/avr.c
	${PROJECT_SOURCE_DIR}/atmega/soc/extmem.c
)

set_source_files_properties(system_tick.S PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

if(CONFIG_STANDALONE)
	set(PORT_SOURCE_FILES
		${PROJECT_SOURCE_DIR}/standalone/avr.c
	)
else()
	set(PORT_SOURCE_FILES
		${FREERTOS_SOURCE_FILES}
	)
endif()

add_library(lwiot-platform ${PORT_SOURCE_FILES} ${AVR_GENERIC_SOURCES})

INSTALL(TARGETS lwiot-platform
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/atmega/include/ DESTINATION include)

if(NOT CONFIG_STANDAONE)
	INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/external/avr-freertos/include/ DESTINATION include)
endif()

include_directories(lwiot-platform PUBLIC ${PROJECT_SOURCE_DIR}/lwiot-core/include ${CMAKE_BINARY_DIR} ${PORT_INCLUDE_DIR})
