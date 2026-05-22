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
/* USB0 = consumer cable. With no breakout / no OTG cable physically
 * forcing host mode via the ID pin, gz's USB code will probably never
 * see an ATTACH on USB0 — it's wired for device mode. USB1 is the dev
 * cable / breakout port and is the natural choice for host. */
#define IQUESYNC_HOST_PORT  VUSB11_PORT_USB1
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
     * bridge thread and unmasks ATTACHEN. */
    if (vusbh11_init(IQUESYNC_HOST_PORT) != VUSB11_OK)
        return -1;

    /* Wait for the bottom-half to finish: ATTACH fires, process_attach
     * does bus reset → speed detect → Chapter-9 enum → INQUIRY/TUR/
     * READ_CAPACITY, then publishes vusbh11_block_size/count. */
    uint32_t waited = 0;
    while (waited < IQUESYNC_ATTACH_TIMEOUT_MS) {
        vusbh11_poll();
        if (vusbh11_block_size == 512 && vusbh11_block_count > 0)
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
