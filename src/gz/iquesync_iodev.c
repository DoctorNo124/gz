/**
 * @file iquesync_iodev.c
 * @brief iodev binding gz's filesystem layer to the iQue USB host driver.
 *
 * Implements struct iodev's probe/disk_init/disk_read/disk_write on top
 * of vusbh11. gz's sys.c calls disk_init then fat_init(&fat, disk_read,
 * disk_write, ...) which gives all of fat.c + POSIX fopen/fread/fwrite
 * access to a FAT-formatted USB stick. Savestate save/load (state.c via
 * gz_macro.c) and macro import/export (gz_macro.c) go through that path
 * unmodified — they just see "a disk is now available."
 *
 * Only built for the iQue OoT-CN target. is_ique() returns false on N64
 * builds, so probe() filters fast there anyway.
 */
#include "z64.h"
#if Z64_VERSION == Z64_OOTIQC

#ifndef USB_HAL_BACKEND_GZ
#  define USB_HAL_BACKEND_GZ
#endif
#include <stddef.h>
#include <stdint.h>
#include "ique.h"
#include "iodev.h"
#include "io.h"
#include "usb_hal.h"
#include "vusb11.h"
#include "vusbh11.h"

#ifndef IQUESYNC_HOST_PORT
/* USB0 = consumer cable. Empirically the working host port on this
 * iQue — USB1 (dev cable / breakout) didn't enumerate devices even
 * with code that was otherwise correct. Likely a hardware issue on
 * the USB1 path (breakout PCB, cable, or controller). USB0 works
 * reliably as host with the device plugged into the consumer port. */
#define IQUESYNC_HOST_PORT  VUSB11_PORT_USB0
#endif

#ifndef IQUESYNC_ATTACH_TIMEOUT_MS
/* How long disk_init waits for ATTACH after bringing up the host stack.
 * 5 s gives the user time to plug a drive in if it wasn't already there.
 * gz blocks on this — too long and gz feels hung at first FAT touch. */
#define IQUESYNC_ATTACH_TIMEOUT_MS  5000u
#endif

static int iquesync_probe(void)
{
    /* Cheap branch: tell gz this iodev only applies on real iQue HW. */
    return is_ique() ? 0 : -1;
}

static int iquesync_disk_init(void)
{
    if (!is_ique())
        return -1;

    /* Mode-agnostic HW enable: MI bit 24 + per-port wrapper clocks.
     * Safe to call once; vusb11_hw_enable() is idempotent. */
    if (vusb11_hw_enable() != VUSB11_OK)
        return -1;

    /* Bring up the host driver on the chosen port. Installs the ISR
     * bridge thread and unmasks ATTACHEN. Idempotent: subsequent calls
     * are no-ops after the first successful init. */
    if (vusbh11_init(IQUESYNC_HOST_PORT) != VUSB11_OK)
        return -1;

    /* Defensive: re-bind USB IRQ → bridge thread queue + re-arm MI mask.
     * After any other gz code that called osSetEventMesg or touched
     * MI_BB_MASK (rdb.c registers fault/break events; menus may
     * disable interrupts during draw), our USB ISR delivery can be
     * broken — telemetry shows isr_count=0, all SETUPs PID=0xFE,
     * controller stuck with TX_SUSPEND_BUSY. usb_hal_irq_rearm()
     * re-registers our queue with libultra's event dispatch table
     * and re-enables MI without recreating the bridge thread. */
    usb_hal_irq_rearm(IQUESYNC_HOST_PORT);

    /* Single-shot enumeration, matching thar0's event-driven design.
     * Retries live where they can actually help:
     *   - SIE hardware (RETRY_DIS cleared) handles transient NAKs
     *   - process_attach's SETUP retry loop (10 attempts with
     *     retry_recovery_soft between) handles a flaky-but-present
     *     device's enumeration
     * The earlier outer 3-attempt loop existed to work around two
     * separate bugs (cold-boot device-warmup; reset-disk re-init) that
     * are now fixed at their root — the 2000ms pre_reset_settle in
     * process_attach and the split-idempotency vusbh11_init. With
     * those, an extra round of port_bringup adds nothing. */
    vusbh11_telem.block_size  = 0;
    vusbh11_telem.block_count = 0;
    vusbh11_telem.attach_pending = true;

    uint32_t waited = 0;
    while (waited < IQUESYNC_ATTACH_TIMEOUT_MS) {
        vusbh11_poll();
        if (vusbh11_telem.block_size == 512 && vusbh11_telem.block_count > 0)
            return 0;
        usb_hal_wait_ms(50);
        waited += 50;
    }
    return -1;
}

static int iquesync_disk_read(size_t lba, size_t n_blocks, void *dst)
{
    return vusbh11_disk_read((uint32_t)lba, (uint32_t)n_blocks, dst);
}

static int iquesync_disk_write(size_t lba, size_t n_blocks, const void *src)
{
    return vusbh11_disk_write((uint32_t)lba, (uint32_t)n_blocks, src);
}

struct iodev iquesync_iodev = {
    .probe       = iquesync_probe,
    .disk_init   = iquesync_disk_init,
    .disk_read   = iquesync_disk_read,
    .disk_write  = iquesync_disk_write,
    /* fifo_* / clock_* / cpu_reset deliberately NULL — gz falls back to
     * its defaults (CP0 COUNT, zu_reset) which work fine on iQue. */
};

#endif /* Z64_VERSION == Z64_OOTIQC */
