/**
 ******************************************************************************
 * @file       pios_wdg.c
 * @author     NinjaPilot, 2026
 * @brief      ESP32 watchdog, backed by the IDF Task WDT.
 *
 * PiOS's watchdog is a flag scheme, not a plain kick: each participating task
 * registers a bit, sets it once per pass, and only when EVERY registered bit
 * has been seen does PIOS_WDG_Clear() actually pet the hardware. That means a
 * single stalled task takes the board down even while the others keep running,
 * which is the behaviour you want from a flight controller.
 *
 * This is deliberately ENABLED on this target. (The ArduPilot ESP32 port
 * disables both the interrupt and task watchdogs in sdkconfig because its loop
 * overruns kept tripping them; we are not carrying that problem, so there is
 * no reason to fly without the protection.)
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

#ifdef PIOS_INCLUDE_WDG

#include "esp_task_wdt.h"
#include "esp_system.h"

static struct wdg_state {
    uint16_t used_flags;        /* bits handed out by RegisterFlag  */
    uint16_t bootup_flags;      /* flags outstanding at last reset  */
    volatile uint16_t active_flags; /* bits set since last Clear    */
    bool     registered;        /* this task subscribed to IDF TWDT */
} wdg;

uint16_t PIOS_WDG_Init(void)
{
    /* A reset cause of TASK_WDT/INT_WDT means the previous boot died with
     * flags outstanding. We cannot recover which ones (that would need RTC
     * memory), so report all-registered and let the System module raise the
     * alarm on a non-zero bootup word. */
    esp_reset_reason_t reason = esp_reset_reason();

    if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT || reason == ESP_RST_WDT) {
        wdg.bootup_flags = 0xFFFF;
        printf("[PIOS] WDG: previous boot ended in a watchdog reset (reason %d)\n",
               (int)reason);
    } else {
        wdg.bootup_flags = 0;
    }

    wdg.used_flags   = 0;
    wdg.active_flags = 0;

    /* NOTE: deliberately NOT subscribing here. PIOS_WDG_Init() runs in the
     * init task, which returns and is deleted once board bring-up finishes
     * (see esp32wroom.c) -- subscribing it would leave the IDF task watchdog
     * pointed at a dead task while the real petting happens elsewhere.
     * PIOS_WDG_Clear() is called from the System module's task, so it
     * subscribes itself on first use. */

    return wdg.bootup_flags;
}

bool PIOS_WDG_RegisterFlag(uint16_t flag_requested)
{
    /* Registration must finish before the scheduler has anything to guard,
     * and PiOS only ever calls this from module init, so no locking. */
    if (wdg.used_flags & flag_requested) {
        return false;
    }
    wdg.used_flags |= flag_requested;
    return true;
}

bool PIOS_WDG_UpdateFlag(uint16_t flag)
{
    wdg.active_flags |= flag;

    if ((wdg.active_flags & wdg.used_flags) == wdg.used_flags) {
        PIOS_WDG_Clear();
        return true;
    }
    return false;
}

uint16_t PIOS_WDG_GetBootupFlags(void)
{
    return wdg.bootup_flags;
}

uint16_t PIOS_WDG_GetActiveFlags(void)
{
    return wdg.active_flags;
}

void PIOS_WDG_Clear(void)
{
    /* Subscribe on first call so the IDF watchdog tracks the task that is
     * actually responsible for petting it -- see the note in
     * PIOS_WDG_Init(). */
    if (!wdg.registered) {
        if (esp_task_wdt_add(NULL) != ESP_OK) {
            /* Nothing useful to do but keep flying; say it once. */
            static bool complained;
            if (!complained) {
                complained = true;
                printf("[PIOS] WDG: could not subscribe to the task watchdog\n");
            }
            wdg.active_flags = 0;
            return;
        }
        wdg.registered = true;
    }

    esp_task_wdt_reset();
    wdg.active_flags = 0;
}

#endif /* PIOS_INCLUDE_WDG */
