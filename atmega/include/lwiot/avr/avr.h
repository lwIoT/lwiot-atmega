/*
 * AVR system header.
 */

#pragma once

#include <lwiot_opts.h>

#ifndef CONFIG_STANDALONE
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#endif
