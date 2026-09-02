/**
 ******************************************************************************
 * @file       pios_servo.c
 * @author     NinjaPilot, 2026
 * @brief      ESP32 servo/ESC output. MCPWM where the SoC has it, LEDC where
 *             it does not (the ESP32-S2).
 *
 * On the original ESP32 and the S3, MCPWM is used rather than LEDC because
 * PIOS_Servo_Update() is a coordinated push -- all channels are meant to take
 * their new value together -- and MCPWM gives us that plus a clean 1us tick.
 * LEDC would be simpler but updates each channel whenever it feels like it.
 *
 * The ESP32-S2 has NO MCPWM at all, so it gets the LEDC backend at the bottom
 * of this file. The selection is on SOC_MCPWM_SUPPORTED, not on a chip name,
 * so a new target picks the right one by itself.
 *
 * The original ESP32 has 2 MCPWM units x 3 timers x 2 generators, so six
 * outputs is the comfortable ceiling. That matches the channel count on the
 * reference wiring and is plenty for a quad.
 *
 * Timer resolution is set to 1MHz, so a comparator value is literally the
 * pulse width in microseconds and PIOS_Servo_Set() is a straight assignment.
 *
 * NOT IMPLEMENTED: DShot. RMT would do it well and it is a few hundred lines,
 * but PWM/OneShot is what a first bring-up needs. MODE is PWM only.
 * @see        The GNU Public License (GPL) Version 3
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

#ifdef PIOS_INCLUDE_SERVO

#include "soc/soc_caps.h"

#define SERVO_DEFAULT_RATE_HZ   400

#if SOC_MCPWM_SUPPORTED

#include "driver/mcpwm_prelude.h"

#define SERVO_TIMEBASE_HZ       1000000UL   /* 1 tick == 1us */
#define SERVO_GENERATORS_PER_TIMER 2

struct servo_chan {
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t  generator;
    uint16_t            position_us;
    uint16_t            pending_us;
};

static struct servo_chan servo_chans[PIOS_ESP32_SERVO_MAX_CHANNELS];
static mcpwm_timer_handle_t servo_timers[PIOS_ESP32_SERVO_MAX_CHANNELS / SERVO_GENERATORS_PER_TIMER];
static uint8_t servo_num_chans;
static uint8_t servo_num_timers;
static uint16_t servo_rate_hz = SERVO_DEFAULT_RATE_HZ;

int32_t PIOS_ESP32_Servo_Init(const struct pios_esp32_servo_cfg *cfg)
{
    PIOS_Assert(cfg);
    PIOS_Assert(cfg->num_pins <= PIOS_ESP32_SERVO_MAX_CHANNELS);

    servo_rate_hz = cfg->default_rate_hz ? cfg->default_rate_hz : SERVO_DEFAULT_RATE_HZ;

    const uint32_t period_ticks = SERVO_TIMEBASE_HZ / servo_rate_hz;

    servo_num_chans  = cfg->num_pins;
    servo_num_timers = (cfg->num_pins + SERVO_GENERATORS_PER_TIMER - 1) /
                       SERVO_GENERATORS_PER_TIMER;

    for (uint8_t t = 0; t < servo_num_timers; t++) {
        mcpwm_timer_config_t tcfg = {
            .group_id      = t / 3,      /* 3 timers per MCPWM group */
            .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = SERVO_TIMEBASE_HZ,
            .period_ticks  = period_ticks,
            .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        };
        if (mcpwm_new_timer(&tcfg, &servo_timers[t]) != ESP_OK) {
            return -1;
        }

        mcpwm_oper_handle_t oper;
        mcpwm_operator_config_t ocfg = { .group_id = tcfg.group_id };

        if (mcpwm_new_operator(&ocfg, &oper) != ESP_OK) {
            return -1;
        }
        if (mcpwm_operator_connect_timer(oper, servo_timers[t]) != ESP_OK) {
            return -1;
        }

        for (uint8_t g = 0; g < SERVO_GENERATORS_PER_TIMER; g++) {
            uint8_t ch = t * SERVO_GENERATORS_PER_TIMER + g;

            if (ch >= cfg->num_pins) {
                break;
            }

            mcpwm_comparator_config_t ccfg = {
                .flags.update_cmp_on_tez = true,
            };
            if (mcpwm_new_comparator(oper, &ccfg, &servo_chans[ch].comparator) != ESP_OK) {
                return -1;
            }

            mcpwm_generator_config_t gcfg = { .gen_gpio_num = cfg->pins[ch] };
            if (mcpwm_new_generator(oper, &gcfg, &servo_chans[ch].generator) != ESP_OK) {
                return -1;
            }

            /* High at counter zero, low when the comparator matches. */
            mcpwm_generator_set_action_on_timer_event(servo_chans[ch].generator,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                             MCPWM_TIMER_EVENT_EMPTY,
                                             MCPWM_GEN_ACTION_HIGH));
            mcpwm_generator_set_action_on_compare_event(servo_chans[ch].generator,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                               servo_chans[ch].comparator,
                                               MCPWM_GEN_ACTION_LOW));

            /* Start at zero width, not at 1000us. An ESC should see no
             * pulse at all until the flight code has actually decided what
             * to output. */
            servo_chans[ch].position_us = 0;
            servo_chans[ch].pending_us  = 0;
            mcpwm_comparator_set_compare_value(servo_chans[ch].comparator, 0);
        }

        if (mcpwm_timer_enable(servo_timers[t]) != ESP_OK) {
            return -1;
        }
        if (mcpwm_timer_start_stop(servo_timers[t], MCPWM_TIMER_START_NO_STOP) != ESP_OK) {
            return -1;
        }
    }

    return 0;
}

void PIOS_Servo_SetHz(const uint16_t *speeds, __attribute__((unused)) const uint32_t *clock,
                      uint8_t banks)
{
    if (!speeds || banks == 0) {
        return;
    }

    /* One rate for every bank: the channels share MCPWM timers in fixed
     * pairs, so per-bank rates would need a different timer assignment than
     * the one built in Init(). Take bank 0's rate and apply it globally. */
    uint16_t rate = speeds[0];

    if (rate == 0) {
        return;
    }
    servo_rate_hz = rate;

    const uint32_t period_ticks = SERVO_TIMEBASE_HZ / rate;

    for (uint8_t t = 0; t < servo_num_timers; t++) {
        mcpwm_timer_set_period(servo_timers[t], period_ticks);
    }
}

void PIOS_Servo_Set(uint8_t servo, uint16_t position)
{
    if (servo >= servo_num_chans) {
        return;
    }
    /* Staged, not applied: PIOS_Servo_Update() is the commit point. */
    servo_chans[servo].pending_us = position;
}

void PIOS_Servo_Update(void)
{
    for (uint8_t ch = 0; ch < servo_num_chans; ch++) {
        if (servo_chans[ch].pending_us == servo_chans[ch].position_us) {
            continue;
        }
        servo_chans[ch].position_us = servo_chans[ch].pending_us;
        /* update_cmp_on_tez means this lands at the next period boundary,
         * so all channels move together and no pulse is ever truncated
         * mid-flight. */
        mcpwm_comparator_set_compare_value(servo_chans[ch].comparator,
                                           servo_chans[ch].position_us);
    }
}

void PIOS_Servo_SetBankMode(__attribute__((unused)) uint8_t bank,
                            __attribute__((unused)) uint8_t mode)
{
    /* Only PWM output mode exists on this backend. */
}

uint8_t PIOS_Servo_GetPinBank(uint8_t pin)
{
    /* Channels are paired onto timers, and a "bank" in PiOS terms is a set
     * of pins that must share a rate. */
    return pin / SERVO_GENERATORS_PER_TIMER;
}

#else /* !SOC_MCPWM_SUPPORTED -- ESP32-S2: LEDC backend */

/*
 * LEDC backend, for parts with no MCPWM (the ESP32-S2).
 *
 * Every channel shares ONE low-speed LEDC timer, which is exactly what PIOS
 * wants here: PIOS_Servo_SetHz() on this port already applies bank 0's rate
 * globally, so there is nothing to gain from separate timers.
 *
 * LEDC duty is a FRACTION of the period, not a microsecond count, so Set()
 * stages microseconds and Update() converts. The S2's LEDC timer is 14 bits;
 * at 400Hz that is 2500us/16384 = 0.15us per step, far finer than any ESC can
 * resolve, so nothing is given up against MCPWM's 1us tick.
 *
 * HONEST DIFFERENCE FROM MCPWM: MCPWM's update_cmp_on_tez makes every channel
 * adopt its new comparator value at the same period boundary. LEDC latches per
 * channel at that channel's next period. Because all channels sit on one timer
 * their periods are aligned, so the worst-case skew is the few microseconds it
 * takes Update() to walk the channels, not a whole period. Fine for PWM ESCs.
 * If DShot or tight sync ever matters on an S2, RMT is the peripheral to use.
 */

#include "driver/ledc.h"

#define SERVO_LEDC_MODE   LEDC_LOW_SPEED_MODE   /* the S2 has no high-speed mode */
#define SERVO_LEDC_TIMER  LEDC_TIMER_0
#define SERVO_LEDC_CLK_HZ 80000000UL            /* APB */

struct servo_chan {
    uint16_t position_us;
    uint16_t pending_us;
};

static struct servo_chan servo_chans[PIOS_ESP32_SERVO_MAX_CHANNELS];
static uint8_t  servo_num_chans;
static uint16_t servo_rate_hz = SERVO_DEFAULT_RATE_HZ;
static uint8_t  servo_duty_bits;

/* Widest duty resolution the APB clock can still divide down to this rate. */
static uint8_t servo_bits_for_rate(uint16_t rate)
{
    uint8_t bits = SOC_LEDC_TIMER_BIT_WIDTH;

    while (bits > 1 && (SERVO_LEDC_CLK_HZ / ((uint32_t)rate << bits)) == 0) {
        bits--;
    }
    return bits;
}

static uint32_t servo_us_to_duty(uint16_t us)
{
    /* 64-bit: us * 2^14 * 400 overflows 32 bits well inside the servo range. */
    return (uint32_t)(((uint64_t)us * ((uint64_t)1 << servo_duty_bits) *
                       (uint64_t)servo_rate_hz) / 1000000ULL);
}

static int32_t servo_apply_timer(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode      = SERVO_LEDC_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = (ledc_timer_bit_t)servo_duty_bits,
        .freq_hz         = servo_rate_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };

    return ledc_timer_config(&tcfg) == ESP_OK ? 0 : -1;
}

int32_t PIOS_ESP32_Servo_Init(const struct pios_esp32_servo_cfg *cfg)
{
    if (!cfg || cfg->num_pins == 0 ||
        cfg->num_pins > PIOS_ESP32_SERVO_MAX_CHANNELS ||
        cfg->num_pins > SOC_LEDC_CHANNEL_NUM) {
        return -1;
    }

    servo_num_chans = cfg->num_pins;
    servo_duty_bits = servo_bits_for_rate(servo_rate_hz);

    if (servo_apply_timer() != 0) {
        return -2;
    }

    for (uint8_t ch = 0; ch < servo_num_chans; ch++) {
        servo_chans[ch].position_us = 0;
        servo_chans[ch].pending_us  = 0;

        ledc_channel_config_t ccfg = {
            .gpio_num   = cfg->pins[ch],
            .speed_mode = SERVO_LEDC_MODE,
            .channel    = (ledc_channel_t)ch,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = SERVO_LEDC_TIMER,
            .duty       = 0,
            .hpoint     = 0,
        };
        if (ledc_channel_config(&ccfg) != ESP_OK) {
            return -3;
        }
    }
    return 0;
}

void PIOS_Servo_SetHz(const uint16_t *speeds, __attribute__((unused)) const uint32_t *clock,
                      uint8_t banks)
{
    if (!speeds || banks == 0 || speeds[0] == 0) {
        return;
    }
    servo_rate_hz   = speeds[0];
    servo_duty_bits = servo_bits_for_rate(servo_rate_hz);

    if (servo_apply_timer() != 0) {
        return;
    }
    /* Duty is a fraction of the period, so a rate change invalidates every
     * channel's duty even though its pulse width in microseconds is the same. */
    for (uint8_t ch = 0; ch < servo_num_chans; ch++) {
        ledc_set_duty(SERVO_LEDC_MODE, (ledc_channel_t)ch,
                      servo_us_to_duty(servo_chans[ch].position_us));
        ledc_update_duty(SERVO_LEDC_MODE, (ledc_channel_t)ch);
    }
}

void PIOS_Servo_Set(uint8_t servo, uint16_t position)
{
    if (servo >= servo_num_chans) {
        return;
    }
    /* Staged, not applied: PIOS_Servo_Update() is the commit point. */
    servo_chans[servo].pending_us = position;
}

void PIOS_Servo_Update(void)
{
    for (uint8_t ch = 0; ch < servo_num_chans; ch++) {
        if (servo_chans[ch].pending_us == servo_chans[ch].position_us) {
            continue;
        }
        servo_chans[ch].position_us = servo_chans[ch].pending_us;
        ledc_set_duty(SERVO_LEDC_MODE, (ledc_channel_t)ch,
                      servo_us_to_duty(servo_chans[ch].position_us));
        ledc_update_duty(SERVO_LEDC_MODE, (ledc_channel_t)ch);
    }
}

void PIOS_Servo_SetBankMode(__attribute__((unused)) uint8_t bank,
                            __attribute__((unused)) uint8_t mode)
{
    /* Only PWM output mode exists on this backend. */
}

uint8_t PIOS_Servo_GetPinBank(__attribute__((unused)) uint8_t pin)
{
    /* One shared timer means one rate for everything: a single bank. */
    return 0;
}

#endif /* SOC_MCPWM_SUPPORTED */

#endif /* PIOS_INCLUDE_SERVO */
