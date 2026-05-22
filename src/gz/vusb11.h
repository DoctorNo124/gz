#ifndef IQUESYNC_VUSB11_H
#define IQUESYNC_VUSB11_H

#include <stdint.h>
#include <stdbool.h>

/*
 * VUSB11 — raw hardware layer for iQue USB controllers.
 *
 * This file is the register reference. Mode-specific bring-up (device or
 * host) lives in higher-level drivers: see vusbh11.h for host-mode bring-up
 * and the device-mode sibling in ~/iquesync for the inverse.
 *
 * Register map derived from SA1 1106 disassembly and corroborated against
 * Thar0's libultra decomp fork (ique-usb branch). The iQue exposes two USB
 * controllers (USB0 / USB1); each occupies a 1 MB MMIO window:
 *
 *   USB0:  0xA490_0000 .. 0xA49F_FFFF   (consumer cable)
 *   USB1:  0xA4A0_0000 .. 0xA4AF_FFFF   (developer cable / breakout)
 *
 * Within each controller window:
 *   +0x00010 .. +0x0001C : iQue-side wrapper regs (init, mode, OTG state,
 *                          OTGCTL — value differs between host and device)
 *   +0x00080 .. +0x000B4 : K20-style USB-OTG core registers (ISTAT, INTEN,
 *                          ERRSTAT, ERREN, STAT, CTL, ADDR, BDTPAGE,
 *                          TOKEN, SOFTHRESHOLD)
 *   +0x000C0 .. +0x00100 : per-endpoint control (16 endpoints)
 *   +0x40010              : per-port wrapper STATUS-enable (write 1 to clock)
 *   +0x80000              : BDT / transfer buffer memory region
 *
 * iQue's MI (MIPS Interface) at 0xA430_003C is MI_HW_INTR_MASK_REG. The
 * USB subsystem hangs off the upper bits:
 *   bit 24       : global USB subsystem enable (SA1 sets this once at boot)
 *   bit 20/22    : per-port "ctlr_init phase" gate (USB0 / USB1)
 *   bit 21/23    : per-port "mode-select phase" gate (USB0 / USB1)
 *   bit 0x400    : USB0 IRQ-pending (iQueBrew)
 *   bit 0x800    : USB1 IRQ-pending (iQueBrew)
 */

/* ----- Controller bases ----- */

#define VUSB11_USB0_BASE       0xA4900000u
#define VUSB11_USB1_BASE       0xA4A00000u
#define VUSB11_REG_SIZE        0x00100000u   /* 1 MB window per controller */

/* iQue-side wrapper offsets (within the controller window). */
#define VUSB11_OFF_REG0C       0x00000Cu  /* ADDINFO: bit 0 = IEHOST status */
#define VUSB11_OFF_REG10       0x000010u  /* wrapper IRQ status (W1C)        */
#define VUSB11_OFF_REG14       0x000014u  /* wrapper IRQ mask                */
#define VUSB11_OFF_OTGSTAT     0x000018u  /* OTG bus state (ID, VBUS, ...)   */
#define VUSB11_OFF_OTGCTL      0x00001Cu  /* OTG control — DPHIGH/DPLOW/...  */

/* USB-OTG core register offsets (within the controller window). */
#define VUSB11_OFF_ISTAT       0x000080u  /* USB transaction IRQ status (W1C) */
#define VUSB11_OFF_INTEN       0x000084u  /* USB IRQ enable mask              */
#define VUSB11_OFF_ERRSTAT     0x000088u  /* error status (W1C)               */
#define VUSB11_OFF_ERREN       0x00008Cu  /* error IRQ enable                 */
#define VUSB11_OFF_STAT        0x000090u  /* last-transaction status          */
#define VUSB11_OFF_CTL         0x000094u  /* control: USB_EN, HOST_MODE, ...  */
#define VUSB11_OFF_ADDR        0x000098u  /* device address + FS/LS select   */
#define VUSB11_OFF_BDTPAGE1    0x00009Cu  /* BDT page bits [15:8]             */
#define VUSB11_OFF_FRMNUML     0x0000A0u  /* frame number low                 */
#define VUSB11_OFF_FRMNUMH     0x0000A4u  /* frame number high                */
#define VUSB11_OFF_TOKEN       0x0000A8u  /* token issue (host mode)          */
#define VUSB11_OFF_SOFTHLD     0x0000ACu  /* SOF threshold                    */
#define VUSB11_OFF_BDTPAGE2    0x0000B0u  /* BDT page bits [23:16]            */
#define VUSB11_OFF_BDTPAGE3    0x0000B4u  /* BDT page bits [31:24]            */
#define VUSB11_OFF_EP_CTL_BASE 0x0000C0u  /* endpoint control area            */
#define VUSB11_MAX_ENDPOINTS   16u

#define VUSB11_OFF_STATUS      0x040010u  /* wrapper STATUS — write 1 to clock */
#define VUSB11_OFF_BDT_MEM     0x080000u  /* in-controller BDT region (DMA-able) */

/* Compatibility aliases for code that still uses the older names. */
#define VUSB11_OFF_REG18       VUSB11_OFF_OTGSTAT
#define VUSB11_OFF_REG1C       VUSB11_OFF_OTGCTL
#define VUSB11_OFF_REG80       VUSB11_OFF_ISTAT
#define VUSB11_OFF_REG84       VUSB11_OFF_INTEN
#define VUSB11_OFF_REG88       VUSB11_OFF_ERRSTAT
#define VUSB11_OFF_REG8C       VUSB11_OFF_ERREN
#define VUSB11_OFF_REG90       VUSB11_OFF_STAT
#define VUSB11_OFF_REG94       VUSB11_OFF_CTL
#define VUSB11_OFF_REG98       VUSB11_OFF_ADDR
#define VUSB11_OFF_REG9C       VUSB11_OFF_BDTPAGE1
#define VUSB11_OFF_REGA8       VUSB11_OFF_TOKEN
#define VUSB11_OFF_REGB0       VUSB11_OFF_BDTPAGE2
#define VUSB11_OFF_REGB4       VUSB11_OFF_BDTPAGE3

/* ----- MI register and per-USB clock-gate bits ----- */

#define MI_HW_INTR_MASK_REG    0xA430003Cu
#define MI_USB_GLOBAL_EN_BIT   0x01000000u   /* bit 24: global USB subsystem enable */
#define MI_USB0_CTLRINIT_BIT   0x00100000u   /* bit 20: USB0 ctlr_init gate */
#define MI_USB1_CTLRINIT_BIT   0x00400000u   /* bit 22: USB1 ctlr_init gate */
#define MI_USB0_MODE_BIT       0x00200000u   /* bit 21: USB0 mode-select gate */
#define MI_USB1_MODE_BIT       0x00800000u   /* bit 23: USB1 mode-select gate */
#define MI_USB0_IRQ_BIT        0x00000400u   /* iQueBrew documented IRQ-pending */
#define MI_USB1_IRQ_BIT        0x00000800u

/* ----- OTGCTL (REG1C) bit definitions (Thar0's usb_hw.h) ----- */

#define VUSB11_OTGCTL_DPHIGH       (1u << 7)  /* D+ pull-up (device mode)   */
#define VUSB11_OTGCTL_RESERVED6    (1u << 6)
#define VUSB11_OTGCTL_DPLOW        (1u << 5)  /* D+ pull-down (host mode)   */
#define VUSB11_OTGCTL_DMLOW        (1u << 4)  /* D- pull-down (host mode)   */
#define VUSB11_OTGCTL_RESERVED3    (1u << 3)  /* iQue-specific (set in host)*/
#define VUSB11_OTGCTL_OTGEN        (1u << 2)  /* OTG enable                 */
#define VUSB11_OTGCTL_RESERVED1    (1u << 1)
#define VUSB11_OTGCTL_RESERVED0    (1u << 0)

/* Mode-specific OTGCTL values, per Thar0's __usbHostMode / __usbDeviceMode. */
#define VUSB11_OTGCTL_HOST_VALUE   (VUSB11_OTGCTL_DPLOW   | VUSB11_OTGCTL_DMLOW | \
                                    VUSB11_OTGCTL_RESERVED3 | VUSB11_OTGCTL_OTGEN)  /* = 0x3C */

/* SA1's device-mode write to REG1C — initial pulse 0x04, then 0x84 (USB0)
 * or 0xB4 (USB1). Kept here for the dual-mode debug-channel use case. */
#define VUSB11_OTGCTL_DEV_INIT     0x04u
#define VUSB11_OTGCTL_DEV_USB0     0x84u
#define VUSB11_OTGCTL_DEV_USB1     0xB4u

/* ----- OTGSTAT (REG18) bits ----- */

#define VUSB11_OTGSTAT_ID            (1u << 7)  /* 0 = Type-A (host), 1 = Type-B (device) */
#define VUSB11_OTGSTAT_ONEMSECEN     (1u << 6)
#define VUSB11_OTGSTAT_LINESTATESTABLE (1u << 5)
#define VUSB11_OTGSTAT_SESS_VLD      (1u << 3)
#define VUSB11_OTGSTAT_BSESSEND      (1u << 2)
#define VUSB11_OTGSTAT_AVBUSVLD      (1u << 0)

/* ----- CTL (REG94) bits ----- */

#define VUSB11_CTL_JSTATE              (1u << 7)
#define VUSB11_CTL_SE0                 (1u << 6)
#define VUSB11_CTL_TX_SUSPEND_BUSY     (1u << 5)
#define VUSB11_CTL_RESET               (1u << 4)
#define VUSB11_CTL_HOST_MODE_EN        (1u << 3)
#define VUSB11_CTL_RESUME              (1u << 2)
#define VUSB11_CTL_ODD_RST             (1u << 1)
#define VUSB11_CTL_USB_EN              (1u << 0)

/* ----- ISTAT (REG80) bits (W1C) ----- */

#define VUSB11_ISTAT_STALL       (1u << 7)
#define VUSB11_ISTAT_ATTACH      (1u << 6)  /* host: device plugged in */
#define VUSB11_ISTAT_RESUME      (1u << 5)
#define VUSB11_ISTAT_SLEEP       (1u << 4)
#define VUSB11_ISTAT_TOKEN_DONE  (1u << 3)
#define VUSB11_ISTAT_SOFTOK      (1u << 2)
#define VUSB11_ISTAT_ERROR       (1u << 1)
#define VUSB11_ISTAT_RST         (1u << 0)
#define VUSB11_ISTAT_ALL         0xFFu

/* ----- INTEN (REG84) bits ----- */

#define VUSB11_INTEN_STALLEN     (1u << 7)
#define VUSB11_INTEN_ATTACHEN    (1u << 6)
#define VUSB11_INTEN_RESUMEEN    (1u << 5)
#define VUSB11_INTEN_SLEEPEN     (1u << 4)
#define VUSB11_INTEN_TOKDNEEN    (1u << 3)
#define VUSB11_INTEN_SOFTOKEN    (1u << 2)
#define VUSB11_INTEN_ERROREN     (1u << 1)
#define VUSB11_INTEN_USBRSTEN    (1u << 0)

/* ----- ADDR (REG98) ----- */
#define VUSB11_ADDR_LSEN         (1u << 7)
#define VUSB11_ADDR_FSEN         0u
#define VUSB11_ADDR_ADDR_MASK    0x7Fu

/* ----- Endpoint control register bits (per-EP, +0xC0 + 4*EP) ----- */
#define VUSB11_EP_HOST_WO_HUB    (1u << 7)  /* host-mode only            */
#define VUSB11_EP_RETRY_DIS      (1u << 6)  /* host-mode only            */
#define VUSB11_EP_CTL_DISABLE    (1u << 4)
#define VUSB11_EP_RX_EN          (1u << 3)
#define VUSB11_EP_TX_EN          (1u << 2)
#define VUSB11_EP_STALL          (1u << 1)
#define VUSB11_EP_HSHK_EN        (1u << 0)

/* ----- STAT (REG90) decoding ----- */
#define VUSB11_STAT_EP(reg)      (((reg) >> 4) & 0xFu)  /* completed EP   */
#define VUSB11_STAT_TX(reg)      (((reg) >> 3) & 1u)    /* 1=TX, 0=RX     */
#define VUSB11_STAT_ODD(reg)     (((reg) >> 2) & 1u)    /* 1=Odd, 0=Even  */

/* ----- TOKEN (REGA8) ----- */
#define VUSB11_TOKEN_OUT(ep)     (((1u)  << 4) | ((ep) & 0xFu))
#define VUSB11_TOKEN_IN(ep)      (((9u)  << 4) | ((ep) & 0xFu))
#define VUSB11_TOKEN_SETUP(ep)   (((13u) << 4) | ((ep) & 0xFu))

/* ----- Per-port BDT physical-address high half (split-written to
 *       BDTPAGE1/2/3 — bytes 1..3 of the 32-bit BDT base address). The
 *       controller can DMA from its own MMIO window. ----- */
#define VUSB11_USB0_BDT_PHYS     0x04980000u
#define VUSB11_USB1_BDT_PHYS     0x04A80000u

/* ----- PI access gate (must read 0xFF before any USB writes) ----- */
#define VUSB11_PI_ALLOWED_IO     0xA4600054u

/* ----- Public API ----- */

typedef enum {
    VUSB11_PORT_USB0 = 0,
    VUSB11_PORT_USB1 = 1
} vusb11_port_t;

typedef enum {
    VUSB11_OK = 0,
    VUSB11_ERR_NOT_BBPLAYER,
    VUSB11_ERR_NO_PI_PERMS,
} vusb11_status_t;

typedef struct {
    bool     present;
    uint32_t status;
} vusb11_diag_t;

/*
 * Mode-agnostic hardware bring-up: enables the global USB clock and both
 * per-port wrappers. Call once before any vusbh11_/vusbdci_ init.
 */
vusb11_status_t vusb11_hw_enable(void);

/* Raw register accessors. The vusbh11/vusbdci layers must use these
 * exclusively (never io_read/io_write to a VUSB11 address directly). */
uint32_t vusb11_read(vusb11_port_t port, uint32_t offset);
void     vusb11_write(vusb11_port_t port, uint32_t offset, uint32_t value);

/* Diagnostic read of the per-port STATUS register. */
vusb11_diag_t vusb11_diagnostic(vusb11_port_t port);

#endif
