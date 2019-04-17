#
# AVR platform build file.
#

include(${PROJECT_SOURCE_DIR}/freertos/freertos.cmake)

set(AVR_GENERIC_SOURCES
	${PROJECT_SOURCE_DIR}/atmega/avr-i2c.c
	${PROJECT_SOURCE_DIR}/atmega/avrspi.c
	${PROJECT_SOURCE_DIR}/atmega/gpio/avr.c

	${PROJECT_SOURCE_DIR}/atmega/avradcchip.cpp
	${PROJECT_SOURCE_DIR}/atmega/system_tick.S
	${PROJECT_SOURCE_DIR}/atmega/avri2calgorithm.cpp
	${PROJECT_SOURCE_DIR}/atmega/avrspibus.cpp
	${PROJECT_SOURCE_DIR}/atmega/avruart.cpp
	${PROJECT_SOURCE_DIR}/atmega/avrwatchdog.cpp
	${PROJECT_SOURCE_DIR}/atmega/gpio/avrgpiochip.cpp

	${PROJECT_SOURCE_DIR}/atmega/soc/avrsystem.cpp
	${PROJECT_SOURCE_DIR}/atmega/soc/cplusplus.cpp
	${PROJECT_SOURCE_DIR}/atmega/soc/critical.c
	${PROJECT_SOURCE_DIR}/atmega/soc/avr.c
)

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

#target_include_directories(lwiot-platform PUBLIC ${PORT_INCLUDE_DIR})
include_directories(lwiot-platform PUBLIC ${PROJECT_SOURCE_DIR}/lwiot-core/include ${CMAKE_BINARY_DIR} ${PORT_INCLUDE_DIR})
