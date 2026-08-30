/**
 ******************************************************************************
 *
 * @file       board_hw_defs.c
 * @author     NinjaPilot, 2026
 * @brief      Hardware descriptor tables for the ESP32-WROOM-32E target.
 *
 * Everything the board physically is lives here; pios_board.c only wires it
 * together. Pin choices follow the esp32buzz reference layout.
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

#include "pios.h"
#ifdef PIOS_INCLUDE_MPU6000
#include <pios_sensors.h>
#include <pios_mpu6000.h>
#endif

/* ---------------------------------------------------------------------- *
 * LED
 *
 * GPIO2 is the LED on essentially every WROOM-32 devkit. If your board has
 * none, this is harmless -- the pin just toggles.
 * ---------------------------------------------------------------------- */
#ifdef PIOS_INCLUDE_LED

static const struct pios_esp32_led board_leds[] = {
    {
        /* GPIO13 is the plain STAT LED on a Thing Plus. GPIO2 there drives a
         * WS2812 RGB LED, which will not respond to a simple level. */
        .pin        = GPIO_NUM_13,
        .active_low = false,
    },
};

const struct pios_esp32_led_cfg pios_led_cfg = {
    .leds     = board_leds,
    .num_leds = NELEMENTS(board_leds),
};

#endif /* PIOS_INCLUDE_LED */

/* ---------------------------------------------------------------------- *
 * SPI -- VSPI (SPI3), sensor bus
 *
 * CS is driven by PiOS as a plain GPIO; see pios_spi.c for why.
 * ---------------------------------------------------------------------- */
#ifdef PIOS_INCLUDE_SPI

static const struct pios_esp32_spi_slave board_spi_sensors_slaves[] = {
    /* slave 0: MPU6000 / ICM-20602
     *
     * CS is GPIO14, NOT GPIO5. On a SparkFun ESP32 Thing Plus, GPIO5 is the
     * onboard microSD card's chip select and it sits on this same SPI bus --
     * using it for the IMU would have the two devices fighting. GPIO5 is fine
     * on a bare WROOM-32 devkit with no SD slot. */
    {
        .cs_pin         = GPIO_NUM_14,
        .mode           = 3,
        /* The MPU6000 only tolerates 1MHz for register access; the burst
         * read of the sensor registers is allowed up to 20MHz. Keep the
         * fast rate conservative for a first bring-up with flying leads --
         * raise it once the wiring is short and known good. */
        .clock_speed_hz = 1000000,
        .fast_speed_hz  = 8000000,
    },
};

const struct pios_esp32_spi_cfg pios_spi_sensors_cfg = {
    .host       = SPI3_HOST,          /* aka VSPI */
    /* Matches the SparkFun ESP32 Thing Plus silkscreen: 5/SCK, 18/MOSI,
     * 19/MISO. NOT the ESP32 VSPI defaults (18/23/19) -- and note the USB-C
     * revision of this board uses those defaults instead, so check the
     * silkscreen rather than the part name. GPIO23 here is 23/SDA. */
    .mosi_pin   = GPIO_NUM_18,
    .miso_pin   = GPIO_NUM_19,
    .sclk_pin   = GPIO_NUM_5,
    .dma_channel = SPI_DMA_CH_AUTO,
    .slaves     = board_spi_sensors_slaves,
    .num_slaves = NELEMENTS(board_spi_sensors_slaves),
};

#endif /* PIOS_INCLUDE_SPI */

/* ---------------------------------------------------------------------- *
 * MPU6000 data-ready
 *
 * GPIO34 is input-only on the ESP32, which makes it a good choice for an
 * interrupt input -- it cannot be accidentally driven.
 *
 * This does NOT run in interrupt context. See pios_exti.c.
 * ---------------------------------------------------------------------- */
#ifdef PIOS_INCLUDE_MPU6000

const struct pios_esp32_exti_cfg pios_exti_mpu6000_cfg = {
    /* GPIO32. GPIO34 is the nicer choice where it exists (input-only, so it
     * cannot be driven by mistake) but it is not broken out on every board;
     * 32 is present on the Thing Plus headers. */
    .pin           = GPIO_NUM_32,
    .vector        = PIOS_MPU6000_IRQHandler,
    .task_stack    = 3072,
    /* Above everything except the timer/IDF internals: a late gyro sample is
     * the one thing this board cannot recover from. */
    .task_priority = configMAX_PRIORITIES - 2,
    .task_name     = "PIOS_MPU6000_DRDY",
};

const struct pios_mpu6000_cfg pios_mpu6000_cfg = {
    .exti_cfg            = NULL,   /* ESP32 uses pios_exti_mpu6000_cfg above */
    .Fifo_store          = PIOS_MPU6000_FIFO_TEMP_OUT | PIOS_MPU6000_FIFO_GYRO_X_OUT |
                           PIOS_MPU6000_FIFO_GYRO_Y_OUT | PIOS_MPU6000_FIFO_GYRO_Z_OUT,
    .Smpl_rate_div_no_dlp = 0,
    .Smpl_rate_div_dlp    = 1,     /* 1kHz internal / (1+1) = 500Hz          */
    .interrupt_cfg        = PIOS_MPU6000_INT_CLR_ANYRD,
    .interrupt_en         = PIOS_MPU6000_INTEN_DATA_RDY,
    .User_ctl             = PIOS_MPU6000_USERCTL_DIS_I2C,
    .Pwr_mgmt_clk         = PIOS_MPU6000_PWRMGMT_PLL_X_CLK,
    .accel_range          = PIOS_MPU6000_ACCEL_8G,
    .gyro_range           = PIOS_MPU6000_SCALE_2000_DEG,
    .filter               = PIOS_MPU6000_LOWPASS_256_HZ,
    .orientation          = PIOS_MPU6000_TOP_0DEG,
    .fast_prescaler       = PIOS_SPI_PRESCALER_4,
    .std_prescaler        = PIOS_SPI_PRESCALER_64,
    .max_downsample       = 1,
};

#endif /* PIOS_INCLUDE_MPU6000 */

/* ---------------------------------------------------------------------- *
 * UARTs
 *
 * UART0 is the one wired to the USB-serial bridge on every devkit, so it is
 * both the boot console and the GCS link. Note the WROOM-32 has no native
 * USB -- unlike the S3 there is no CDC option, the bridge is the only path.
 * ---------------------------------------------------------------------- */
#ifdef PIOS_INCLUDE_USART

const struct pios_esp32_usart_cfg pios_usart_telem_cfg = {
    .port           = UART_NUM_0,
    .rx_pin         = GPIO_NUM_3,
    .tx_pin         = GPIO_NUM_1,
    .init_baud      = 115200,
    .rx_buffer_size = 512,
    .tx_buffer_size = 512,
    .invert_rx      = false,
};

/* Spare port: GPS, a second telemetry link, or serial RC.
 * For SBUS set .invert_rx = true -- the ESP32 UART inverts in hardware, so
 * no external inverter is needed. */
const struct pios_esp32_usart_cfg pios_usart_aux_cfg = {
    .port           = UART_NUM_2,
    .rx_pin         = GPIO_NUM_16,
    .tx_pin         = GPIO_NUM_17,
    .init_baud      = 57600,
    .rx_buffer_size = 512,
    .tx_buffer_size = 256,
    .invert_rx      = false,
};

#endif /* PIOS_INCLUDE_USART */

/* ---------------------------------------------------------------------- *
 * Servo outputs (MCPWM)
 *
 * Quad X order matches the mixer's expectation of channels 1-4.
 * ---------------------------------------------------------------------- */
#ifdef PIOS_INCLUDE_SERVO

/* GPIO32 is now the IMU data-ready input, and 21/22 are the Qwiic I2C pins on
 * a Thing Plus, so the servo set is trimmed to four -- which is all a quad
 * needs. Put them back if you are on a bare devkit and want 6 channels. */
/* Motor outputs, quad X order (M1..M4).
 *
 * Every one of these is directly labelled on the Thing Plus silkscreen, which
 * matters: GPIO25/26 are only reachable as A1/A0 and are not labelled by
 * number, so they are avoided. GPIO12 is skipped deliberately -- it is the
 * flash-voltage strapping pin, and an ESC holding it at power-up stops the
 * board booting. */
static const gpio_num_t board_servo_pins[] = {
    GPIO_NUM_15,   /* M1 */
    GPIO_NUM_33,   /* M2 */
    GPIO_NUM_27,   /* M3 */
    GPIO_NUM_17,   /* M4 */
};

const struct pios_esp32_servo_cfg pios_servo_cfg = {
    .pins            = board_servo_pins,
    .num_pins        = NELEMENTS(board_servo_pins),
    /* 400Hz suits multirotor ESCs. Actuator settings can change this at
     * runtime via PIOS_Servo_SetHz(). */
    .default_rate_hz = 400,
};

#endif /* PIOS_INCLUDE_SERVO */

/* ---------------------------------------------------------------------- *
 * RC input -- PPM sum, decoded by RMT
 * ---------------------------------------------------------------------- */
#ifdef PIOS_INCLUDE_PPM

const struct pios_esp32_ppm_cfg pios_ppm_cfg = {
    /* GPIO21. GPIO4 is not broken out on the Thing Plus headers. Avoided
     * GPIO12/15 deliberately -- both are strapping pins (12 selects the flash
     * voltage) and an RC receiver holding one at boot can brick a power-up. */
    .pin = GPIO_NUM_21,
};

#endif /* PIOS_INCLUDE_PPM */
