#include "z64.h"
#if Z64_VERSION == Z64_OOTIQC

#ifndef USB_HAL_BACKEND_GZ
#  define USB_HAL_BACKEND_GZ
#endif
#include "usb_hal.h"
#include "vusb11.h"

/*
 * VUSB11 — raw hardware layer.
 *
 * This file is intentionally tiny. It does only two things:
 *   1. Provides register access wrappers.
 *   2. Implements vusb11_hw_enable(): the mode-agnostic prelude that SA1's
 *      __usbHwInit also performs — set MI bit 24 (global USB enable),
 *      then write STATUS=1 to BOTH per-port wrappers so their bus paths
 *      are clocked.
 *
 * Everything else (controller-mode selection, BDT setup, interrupt handling)
 * belongs to a higher-level driver: vusbh11.c for host mode.
 *
 * Dependencies are limited to usb_hal.h — no libdragon / libultra symbols
 * directly. Pick a usb_hal_<env>.c at link time.
 */

static uint32_t reg_base(vusb11_port_t port)
{
    return (port == VUSB11_PORT_USB0) ? VUSB11_USB0_BASE : VUSB11_USB1_BASE;
}

uint32_t vusb11_read(vusb11_port_t port, uint32_t offset)
{
    return usb_hal_io_read(reg_base(port) + offset);
}

void vusb11_write(vusb11_port_t port, uint32_t offset, uint32_t value)
{
    usb_hal_io_write(reg_base(port) + offset, value);
}

vusb11_diag_t vusb11_diagnostic(vusb11_port_t port)
{
    /* NOTE: do NOT read VUSB11_OFF_STATUS (+0x40010). Empirically that
     * register stalls the PI bus when read, even after writing 1 to it.
     * iquesync defined this helper but never called it; we keep it as a
     * stub so the API remains stable, but it only reports register-base
     * presence based on PI permissions. */
    uint32_t pi = usb_hal_io_read(VUSB11_PI_ALLOWED_IO) & 0xFFu;
    vusb11_diag_t d = {
        .present = (pi == 0xFFu),
        .status  = 0u,
    };
    usb_hal_log("vusb11_diagnostic[USB%d]: PI_ALLOWED_IO=0x%02lx (%s)\n",
                (int)port, (unsigned long)pi,
                d.present ? "present" : "PI denied");
    return d;
}

vusb11_status_t vusb11_hw_enable(void)
{
    if (!usb_hal_is_ique()) {
        usb_hal_log("vusb11_hw_enable: not running on iQue\n");
        return VUSB11_ERR_NOT_BBPLAYER;
    }

    /* Sanity check PI permissions. If this reads 0x00 the SK didn't apply
     * our requested hwAccessRights and any PI write to a USB register will
     * hang the bus. Don't abort — homebrew flow may still let the writes
     * through — but warn loudly. */
    uint32_t pi_allowed = usb_hal_io_read(VUSB11_PI_ALLOWED_IO) & 0xFFu;
    if (pi_allowed != 0xFFu) {
        usb_hal_log("vusb11_hw_enable: PI_ALLOWED_IO=0x%02lx (expect 0xFF)\n",
                    (unsigned long)pi_allowed);
    }

    /* (Originally we wrote 0x01000000 to MI 0x3C here "to mirror SA1's
     * known-good sequence". In libdragon's MI_BB_MASK encoding that's
     * MI_BB_WMASK_CLR_BTN — which CLEARS THE POWER-BUTTON IRQ MASK.
     * SA1 doesn't care because it owns the box; gz running under OoT
     * absolutely needs the power button to keep working so the user
     * can shut down. Confirmed live: with this write present, the
     * iQue power button becomes unresponsive after any USB activity
     * and the user has to pull the AC cord. Removed.) */

    /* SA1's __usbHwInit unconditionally writes 1 to BOTH wrapper STATUS
     * registers (USB0 at 0xA4940010 and USB1 at 0xA4A40010) regardless of
     * which port is being used. These are the iQue-side bus wrappers
     * between MI and the USB controller IP; without both enabled the
     * controller's DMA path to system RAM is gated off, causing persistent
     * DMAERR. The register is WRITE-ONLY — reading it back will stall PI. */
    usb_hal_io_write(VUSB11_USB0_BASE + VUSB11_OFF_STATUS, 1);
    usb_hal_io_write(VUSB11_USB1_BASE + VUSB11_OFF_STATUS, 1);

    return VUSB11_OK;
}

#endif /* Z64_VERSION == Z64_OOTIQC */
