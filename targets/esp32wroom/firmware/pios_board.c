/**
 ******************************************************************************
 *
 * @file       pios_board.c
 * @author     NinjaPilot, 2026
 * @brief      Board initialisation for the ESP32-WROOM-32E target.
 *
 * Call order mirrors the CopterControl board file, minus everything that has
 * no ESP32 analogue (IAP, internal-flash settings FS, USB, RTC, timer clock
 * setup -- IDF owns all of those or they are not implemented yet).
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

#include "inc/openpilot.h"
#include <pios_board_info.h>
#include <uavobjectsinit.h>
#include <hwsettings.h>
#include <manualcontrolsettings.h>
#include <mixersettings.h>
#include <actuatorsettings.h>
#include <taskinfo.h>
#include <pios_com_priv.h>
#include <pios_rcvr_priv.h>
#include <pios_flashfs.h>
#include <pios_board_info.h>
#include <pios_debuglog.h>

#include "../board_hw_defs.c"

/* Externally visible COM handles, referenced by the Telemetry module.
 * Declared in pios_board.h as PIOS_COM_TELEM_RF. */
uint32_t pios_com_telem_rf_id;
uint32_t pios_com_aux_id;

/* Receiver group map, indexed by MANUALCONTROLSETTINGS_CHANNELGROUPS_*. */
uint32_t pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE];

/* Sensor bus handles. */
static uint32_t pios_spi_sensors_id;

/* ---------------------------------------------------------------------- *
 * Placeholders for the settings filesystem, which this target does not have
 * yet (see pios_config.h).
 *
 * systemmod.c reads both ids and only calls PIOS_FLASHFS_GetStats() when one
 * is non-zero, so leaving them at 0 disables that reporting cleanly. The stub
 * exists purely to satisfy the linker; if it is ever actually reached, that
 * means an id got set without a real backend behind it, so it says so rather
 * than returning plausible zeros.
 * ---------------------------------------------------------------------- */
/* pios/common/pios_board_info.c is not built for this target: it places the
 * blob in a .boardinfo section that only the PiOS bootloader's linker script
 * defines, and it derives fw_base/fw_size from BL_/FW_BANK values that have no
 * meaning here (ESP-IDF's bootloader and partition table own the flash map).
 * Define it directly instead.
 *
 * board_rev MUST stay 0x02 -- see the comment in board-info.mk. */
const struct pios_board_info pios_board_info_blob = {
    .magic      = PIOS_BOARD_INFO_BLOB_MAGIC,
    .board_type = 0x12,
    .board_rev  = 0x02,
    .bl_rev     = 0x00,   /* no PiOS bootloader on this target */
    .hw_type    = 0x00,
    .fw_base    = 0x00010000,  /* factory app partition, see partitions.csv */
    .fw_size    = 1024 * 1024,
    .desc_base  = 0,
    .desc_size  = 0,
    .ee_base    = 0,
    .ee_size    = 0,
};

/* uavobjectmanager.c calls this unconditionally. The real implementation
 * (pios/common/pios_debuglog.c) logs to PIOS_FLASHFS, which this target does
 * not have yet -- so the on-board DebugLog recorder is NOT available here.
 * Worth knowing, because it is the flight recorder the rest of this project
 * leans on. */
void PIOS_DEBUGLOG_UAVObject(__attribute__((unused)) uint32_t objid,
                             __attribute__((unused)) uint16_t instid,
                             __attribute__((unused)) size_t size,
                             __attribute__((unused)) uint8_t *data)
{}

uintptr_t pios_uavo_settings_fs_id;
uintptr_t pios_user_fs_id;

int32_t PIOS_FLASHFS_GetStats(uintptr_t fs_id, struct PIOS_FLASHFS_Stats *stats);
int32_t PIOS_FLASHFS_GetStats(__attribute__((unused)) uintptr_t fs_id,
                              __attribute__((unused)) struct PIOS_FLASHFS_Stats *stats)
{
    printf("[BOARD] PIOS_FLASHFS_GetStats called with no filesystem backend\n");
    return -1;
}

/**
 * Bring up a COM device on one of the ESP32 UARTs.
 *
 * The COM layer owns its own ring buffers; the IDF UART driver has separate
 * ones underneath. That double-buffering is a little wasteful but it keeps
 * pios_usart.c able to hand up whole blocks instead of single bytes.
 */
static void board_com_init(uint32_t *com_id,
                           const struct pios_esp32_usart_cfg *usart_cfg,
                           uint16_t rx_len, uint16_t tx_len)
{
    uint32_t usart_id;

    if (PIOS_ESP32_USART_Init(&usart_id, usart_cfg) != 0) {
        PIOS_Assert(0);
    }

    uint8_t *rx_buffer = (uint8_t *)pios_malloc(rx_len);
    uint8_t *tx_buffer = (uint8_t *)pios_malloc(tx_len);

    PIOS_Assert(rx_buffer);
    PIOS_Assert(tx_buffer);

    if (PIOS_COM_Init(com_id, &pios_esp32_usart_com_driver, usart_id,
                      rx_buffer, rx_len, tx_buffer, tx_len) != 0) {
        PIOS_Assert(0);
    }
}

/**
 * PIOS_Board_Init()
 *
 * Runs from the init task, with the scheduler already going (ESP-IDF starts
 * it before app_main).
 */

/* ---------------------------------------------------------------------- *
 * Default airframe: Quad X
 *
 * This target has no settings filesystem (see the PIOS_FLASHFS stubs above),
 * so anything the GCS writes lives in RAM and is gone at the next power-up.
 * A quad whose mixer evaporates on reboot is worse than no mixer at all, so
 * the Quad X configuration is compiled in here and reapplied every boot. The
 * GCS can still change any of it at runtime -- the change just will not
 * survive a reset until there is a real settings backend behind
 * pios_uavo_settings_fs_id.
 * ---------------------------------------------------------------------- */
static void board_apply_default_airframe(void)
{
    MixerSettingsData mixer;
    ActuatorSettingsData act;

    MixerSettingsGet(&mixer);

    /* The stock ThrottleCurve1 default is all zeros, so even a perfectly
     * correct mixer commands nothing at any stick position. Linear 0..100%. */
    mixer.ThrottleCurve1[0] = 0.0f;
    mixer.ThrottleCurve1[1] = 0.25f;
    mixer.ThrottleCurve1[2] = 0.5f;
    mixer.ThrottleCurve1[3] = 0.75f;
    mixer.ThrottleCurve1[4] = 1.0f;

    /*
     * Quad X, matching the table the GCS itself writes for this airframe
     * (xMixer in configmultirotorwidget.cpp), scaled by 127. Yaw is negative
     * for a CW prop and positive for CCW, so the diagonals pair up:
     *
     *   M1  front-left   GPIO15   pitch +1  roll +1  yaw -1   CW
     *   M2  front-right  GPIO33   pitch +1  roll -1  yaw +1   CCW
     *   M3  rear-right   GPIO27   pitch -1  roll -1  yaw -1   CW
     *   M4  rear-left    GPIO12   pitch -1  roll +1  yaw +1   CCW
     *
     * "Front" is whichever way the IMU's +X points, NOT the ESP32 board --
     * the sensor is on flying leads, so fix its orientation to the frame
     * before trusting any of this.
     */
    mixer.Mixer1Type = MIXERSETTINGS_MIXER1TYPE_MOTOR;
    mixer.Mixer1Vector.ThrottleCurve1 = 127;
    mixer.Mixer1Vector.ThrottleCurve2 = 0;
    mixer.Mixer1Vector.Roll  =  127;
    mixer.Mixer1Vector.Pitch =  127;
    mixer.Mixer1Vector.Yaw   = -127;

    mixer.Mixer2Type = MIXERSETTINGS_MIXER2TYPE_MOTOR;
    mixer.Mixer2Vector.ThrottleCurve1 = 127;
    mixer.Mixer2Vector.ThrottleCurve2 = 0;
    mixer.Mixer2Vector.Roll  = -127;
    mixer.Mixer2Vector.Pitch =  127;
    mixer.Mixer2Vector.Yaw   =  127;

    mixer.Mixer3Type = MIXERSETTINGS_MIXER3TYPE_MOTOR;
    mixer.Mixer3Vector.ThrottleCurve1 = 127;
    mixer.Mixer3Vector.ThrottleCurve2 = 0;
    mixer.Mixer3Vector.Roll  = -127;
    mixer.Mixer3Vector.Pitch = -127;
    mixer.Mixer3Vector.Yaw   = -127;

    mixer.Mixer4Type = MIXERSETTINGS_MIXER4TYPE_MOTOR;
    mixer.Mixer4Vector.ThrottleCurve1 = 127;
    mixer.Mixer4Vector.ThrottleCurve2 = 0;
    mixer.Mixer4Vector.Roll  =  127;
    mixer.Mixer4Vector.Pitch = -127;
    mixer.Mixer4Vector.Yaw   =  127;

    MixerSettingsSet(&mixer);

    ActuatorSettingsGet(&act);
    for (uint8_t i = 0; i < 4; i++) {
        act.ChannelType[i]    = ACTUATORSETTINGS_CHANNELTYPE_PWM;
        act.ChannelAddr[i]    = i;
        /* Min == Neutral means the motors sit at their stop when armed.
         * MotorsSpinWhileArmed stays FALSE. Raise Neutral only after the
         * ESCs have been calibrated against these endpoints. */
        act.ChannelMin[i]     = 1000;
        act.ChannelNeutral[i] = 1000;
        act.ChannelMax[i]     = 2000;
    }
    /* 50Hz is the rate every analogue ESC understands. The MCPWM driver
     * applies one rate to all channels (see PIOS_Servo_SetHz), so this is
     * effectively a single global setting -- raise it once you know what the
     * ESCs accept. */
    act.BankUpdateFreq[0] = 50;
    ActuatorSettingsSet(&act);
}

void PIOS_Board_Init(void)
{
    PIOS_DELAY_Init();

    PIOS_LED_Init(&pios_led_cfg);
    PIOS_LED_On(PIOS_LED_HEARTBEAT);

    /* The task monitor owns a recursive mutex that the System module's
     * updateStats() takes once a second via
     * PIOS_TASK_MONITOR_GetIdlePercentage(). Skipping this init does not
     * fail loudly at startup -- it asserts inside FreeRTOS a second later,
     * once the System task first runs. */
    if (PIOS_TASK_MONITOR_Initialize(TASKINFO_RUNNING_NUMELEM)) {
        PIOS_Assert(0);
    }

    /* Object plumbing has to exist before anything tries to publish. */
    PIOS_CALLBACKSCHEDULER_Initialize();
    EventDispatcherInitialize();
    UAVObjInitialize();
    UAVObjectsInitializeAll();

    /* Compiled-in Quad X mixer and output endpoints -- see the comment on the
     * function for why this is not left to the GCS. */
    board_apply_default_airframe();

    /* No settings filesystem on this target yet (see pios_config.h), so
     * HwSettings comes up on defaults every boot. Say so once, plainly --
     * a board that silently forgets its configuration on every power cycle
     * is a bad surprise to discover mid-tuning. */
    HwSettingsInitialize();
    printf("[BOARD] settings are NOT persistent on this target yet "
           "(no NVS backend); configure over the GCS link each boot\n");

    PIOS_WDG_Init();

    AlarmsInitialize();

    /* --- Telemetry / console ------------------------------------------ */
    board_com_init(&pios_com_telem_rf_id, &pios_usart_telem_cfg,
                   PIOS_COM_TELEM_RF_RX_BUF_LEN, PIOS_COM_TELEM_RF_TX_BUF_LEN);

    /* --- Is anything actually on the sensor bus? -----------------------
     *
     * Before SPI claims MISO, drive the pin's internal pull-down and then its
     * pull-up and read it back. A connected, powered device holds the line;
     * an unconnected one follows whichever pull is enabled. This separates
     * "wired wrong" from "not powered" without a meter, and a WHO_AM_I of
     * 0xFF alone cannot tell those apart. */
    {
        gpio_config_t io = {
            .intr_type    = GPIO_INTR_DISABLE,
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = 1ULL << GPIO_NUM_19,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
        };
        gpio_config(&io);
        vTaskDelay(pdMS_TO_TICKS(5));
        int with_pd = gpio_get_level(GPIO_NUM_19);

        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io.pull_up_en   = GPIO_PULLUP_ENABLE;
        gpio_config(&io);
        vTaskDelay(pdMS_TO_TICKS(5));
        int with_pu = gpio_get_level(GPIO_NUM_19);

        const char *verdict =
            (with_pd == 1) ? "held HIGH by something -- device present and powered" :
            (with_pu == 0) ? "held LOW by something -- device present, but check CS" :
                             "FLOATING -- nothing connected or the sensor has no power";
        printf("[BOARD] MISO(GPIO19) pulldown=%d pullup=%d : %s\n",
               with_pd, with_pu, verdict);
    }

    /* --- Sensor bus --------------------------------------------------- */
    if (PIOS_ESP32_SPI_Init(&pios_spi_sensors_id, &pios_spi_sensors_cfg) != 0) {
        PIOS_Assert(0);
    }


#ifdef PIOS_INCLUDE_MPU6000
    /* Slave 0 on the sensor bus.
     *
     * Two things worth knowing here:
     *
     * 1. PIOS_MPU6000_Init() returns 0 as long as its allocation succeeds.
     *    PIOS_MPU6000_Config() calls PIOS_MPU6000_Test() but ignores the
     *    result, so a missing or unrecognised part does NOT show up in the
     *    return value. WHO_AM_I is read separately below to report it.
     *
     * 2. PIOS_MPU6000_Register() and the data-ready task are started
     *    REGARDLESS of whether a sensor answered. That looks wrong, and it
     *    is what an earlier revision "fixed" -- but skipping the registration
     *    hangs module init: the sensor consumers wait on a PIOS_SENSORS
     *    instance that never appears, and the board never reaches telemetry.
     *    A board that boots, reports the fault over UAVTalk and refuses to
     *    arm is far more useful than one that silently wedges, so register
     *    unconditionally and let the alarm carry the bad news.
     */
    PIOS_MPU6000_Init(pios_spi_sensors_id, 0, &pios_mpu6000_cfg);

    int32_t imu_id = PIOS_MPU6000_ReadID();

    if (imu_id != 0x68 && imu_id != 0x70 && imu_id != 0x12) {
        printf("[BOARD] no usable IMU on SPI3 (SCLK=5 MOSI=18 MISO=19 CS=14): WHO_AM_I=0x%02X "
               "(expect 0x68 MPU6000/6050, 0x70 MPU6500, 0x12 ICM-20602; "
               "0x00/0xFF means nothing is answering)\n", (unsigned)imu_id);
        AlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_CRITICAL);
    } else {
        printf("[BOARD] IMU found, WHO_AM_I=0x%02X\n", (unsigned)imu_id);
    }

    PIOS_MPU6000_Register();

    /* The data-ready path runs in a task, not an ISR -- see
     * pios/esp32/pios_exti.c for the two reasons why. */
    if (PIOS_ESP32_EXTI_Init(&pios_exti_mpu6000_cfg) != 0) {
        printf("[BOARD] failed to start the MPU6000 data-ready task\n");
        AlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_CRITICAL);
    }
#endif /* PIOS_INCLUDE_MPU6000 */

    /* --- Actuator outputs --------------------------------------------- */
#ifdef PIOS_INCLUDE_SERVO
    if (PIOS_ESP32_Servo_Init(&pios_servo_cfg) != 0) {
        PIOS_Assert(0);
    }
#endif

    /* --- RC input ------------------------------------------------------ */
#ifdef PIOS_INCLUDE_PPM
    {
        uint32_t pios_ppm_id;

        if (PIOS_ESP32_PPM_Init(&pios_ppm_id, &pios_ppm_cfg) != 0) {
            PIOS_Assert(0);
        }

        uint32_t pios_ppm_rcvr_id;

        if (PIOS_RCVR_Init(&pios_ppm_rcvr_id, &pios_esp32_ppm_rcvr_driver,
                           pios_ppm_id) != 0) {
            PIOS_Assert(0);
        }
        pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PPM] = pios_ppm_rcvr_id;
    }
#endif /* PIOS_INCLUDE_PPM */

    PIOS_LED_Off(PIOS_LED_HEARTBEAT);
}
