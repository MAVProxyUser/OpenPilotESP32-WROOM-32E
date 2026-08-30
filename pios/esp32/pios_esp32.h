/**
 ******************************************************************************
 *
 * @file       pios_esp32.h
 * @author     NinjaPilot, 2026
 * @brief      Top-level PiOS header for the ESP32 (Xtensa LX6) architecture.
 *
 * This mirrors the role pios_sim_posix.h plays for the POSIX targets: pios.h
 * keys the whole architecture selection off USE_<BOARD>, and this file is the
 * ESP32 branch. Nothing STM32 is reachable from here.
 *
 * The important structural difference from the STM32 targets is that we run on
 * ESP-IDF's own FreeRTOS build, not the kernel vendored under
 * pios/common/libraries/FreeRTOS. ESP-IDF owns FreeRTOSConfig.h (it is
 * generated from sdkconfig), owns the linker script, and has already started
 * the scheduler by the time our entry point runs. See esp32wroom.c.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef PIOS_ESP32_H
#define PIOS_ESP32_H

/* PIOS board specific feature selection */
#include "pios_config.h"

/* C Lib includes */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

/* ESP-IDF's FreeRTOS. Note the path prefix: IDF headers live under
 * freertos/ and pulling them in any other way picks up the vendored
 * V11.3.0 kernel instead, which will not link against IDF's port. */
#ifdef PIOS_INCLUDE_FREERTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#endif

#include <pios_mem.h>
#include <pios_architecture.h>

/* PIOS board specific device configuration */
#include "pios_board.h"

/* PIOS debug interface */
#include <pios_debug.h>
#include <pios_debuglog.h>

/* PIOS common functions */
#include <pios_crc.h>

#ifdef PIOS_INCLUDE_TASK_MONITOR
#ifndef PIOS_INCLUDE_FREERTOS
#error PiOS Task Monitor requires PIOS_INCLUDE_FREERTOS to be defined
#endif
#include <pios_task_monitor.h>
/* Not stock FreeRTOS -- see pios/esp32/pios_task_runtime.c. */
UBaseType_t uxTaskGetRunTime(TaskHandle_t xTask);
#endif

#ifdef PIOS_INCLUDE_CALLBACKSCHEDULER
#ifndef PIOS_INCLUDE_FREERTOS
#error PiOS CallbackScheduler requires PIOS_INCLUDE_FREERTOS to be defined
#endif
#include <pios_callbackscheduler.h>
#endif

/* PIOS system functions */
#ifdef PIOS_INCLUDE_DELAY
#include <pios_delay.h>
#include <pios_deltatime.h>
#endif

#include "pios_initcall.h"

#ifdef PIOS_INCLUDE_SYS
#include <pios_sys.h>
#endif

#ifdef PIOS_INCLUDE_IRQ
#include <pios_irq.h>
#endif

#ifdef PIOS_INCLUDE_WDG
#include <pios_wdg.h>
#endif

#ifdef PIOS_INCLUDE_LED
#include <pios_led.h>
#endif

#ifdef PIOS_INCLUDE_SPI
#include <pios_spi.h>
#endif

#ifdef PIOS_INCLUDE_I2C
/* Safe to include here -- unlike pios_sensors.h this pulls in nothing but
 * stdbool, so there is no cycle back through pios.h. */
#include <pios_i2c.h>
#endif

#ifdef PIOS_INCLUDE_SERVO
#include <pios_servo.h>
#endif

#ifdef PIOS_INCLUDE_RCVR
#include <pios_rcvr.h>
#endif

#ifdef PIOS_INCLUDE_PPM
#include <pios_ppm.h>
#endif

#ifdef PIOS_INCLUDE_COM
#include <pios_com.h>
#endif

/* Sensor headers are deliberately NOT included here. pios_sensors.h opens
 * with #include <pios.h>, so pulling it (or pios_mpu6000.h, which needs its
 * typedefs) in from this file creates a cycle: pios_sensors.c -> pios_sensors.h
 * -> pios.h -> here -> pios_mpu6000.h, at which point pios_sensors.h's guard
 * is set but PIOS_SENSORS_Driver is not yet defined.
 *
 * The STM32 branch of pios.h does not include them either -- board files pull
 * in the sensor drivers they actually use. board_hw_defs.c does the same. */

/* The shared sensor drivers carry a `const struct pios_exti_cfg *` field
 * describing an STM32 EXTI line. We never populate it (see pios_exti.c), but
 * it has to be a known -- if incomplete -- type for the headers to compile. */
struct pios_exti_cfg;
extern int32_t PIOS_EXTI_Init(const struct pios_exti_cfg *cfg);

/* ESP32-specific driver private interfaces (config structs + Init
 * prototypes). These deliberately do NOT reuse pios/inc/pios_*_priv.h --
 * those carry STM32 peripheral-library types (TIM_TimeBaseInitTypeDef,
 * GPIO_InitTypeDef) and are unusable here. The POSIX backend takes the
 * same approach. */
#include <pios_esp32_priv.h>

#endif /* PIOS_ESP32_H */
