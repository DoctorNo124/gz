#ifndef IQUESYNC_VUSBH11_H
#define IQUESYNC_VUSBH11_H

#include <stdint.h>
#include "vusb11.h"

/*
 * VUSBH11 — USB host-mode driver for the iQue.
 *
 * Mirrors the bring-up that Thar0's libultra decomp (ique-usb branch)
 * performs in __usbCtlrInit + __usbHostMode for the host-side controller.
 *
 *   1. vusb11_hw_enable() must have been called first (it sets MI 0x3C bit
 *      24 and clocks both per-port wrappers — required regardless of mode).
 *   2. vusbh11_init(port) then performs the host-mode controller bring-up
 *      on a single port, leaves the ATTACH interrupt unmasked, and returns.
 *   3. Plugging a USB device should fire ATTACH; the ISR (vusbh11_isr) is
 *      already registered with libdragon so it'll log the event.
 *
 * No transactions are issued by this layer. Higher-level USB Chapter-9
 * code (host_ch9_get_descriptor, set_address, set_configuration) goes on
 * top — not yet written.
 */

/* Telemetry — observable by main.c for the status screen. */
extern volatile uint32_t vusbh11_isr_count;
extern volatile uint32_t vusbh11_attach_count;
extern volatile uint32_t vusbh11_rst_count;
extern volatile uint32_t vusbh11_tokdne_count;
extern volatile uint32_t vusbh11_softok_count;
extern volatile uint32_t vusbh11_error_count;
extern volatile uint32_t vusbh11_last_istat;
extern volatile uint32_t vusbh11_last_errstat;
extern volatile uint32_t vusbh11_last_otgstat;
extern volatile uint32_t vusbh11_last_reg10;

/* Set by the ISR when ATTACH first fires; ATTACHEN is masked at the same
 * time so the IRQ stops re-asserting. Bottom-half code in the main loop
 * should consume this flag and run bus reset / speed detect / enumeration. */
extern volatile bool vusbh11_attach_pending;

typedef enum {
    VUSBH11_SPEED_UNKNOWN = 0,
    VUSBH11_SPEED_LOW,
    VUSBH11_SPEED_FULL,
} vusbh11_speed_t;

extern volatile vusbh11_speed_t vusbh11_device_speed;

/* GET_DESCRIPTOR(DEVICE,8) result. Populated after the IN data phase
 * succeeds — first 8 bytes of the device descriptor are copied here.
 * Layout:
 *   [0] bLength            (should be 0x12 = 18)
 *   [1] bDescriptorType    (should be 0x01 = DEVICE)
 *   [2..3] bcdUSB          (USB version, little-endian)
 *   [4] bDeviceClass
 *   [5] bDeviceSubClass
 *   [6] bDeviceProtocol
 *   [7] bMaxPacketSize0    (8, 16, 32 or 64)
 * VID/PID live at bytes 8..11 — a second GET_DESCRIPTOR with wLength=18
 * is needed to read those, and per USB spec that should happen at a
 * non-default address (after SET_ADDRESS). Not done by this layer yet. */
extern volatile uint8_t  vusbh11_dev_desc[8];
extern volatile bool     vusbh11_dev_desc_valid;

typedef enum {
    VUSBH11_ENUM_IDLE = 0,
    VUSBH11_ENUM_SETUP_SENT,
    VUSBH11_ENUM_IN_SENT,
    VUSBH11_ENUM_DONE,
    VUSBH11_ENUM_ERROR,
} vusbh11_enum_state_t;

extern volatile vusbh11_enum_state_t vusbh11_enum_state;

vusb11_status_t vusbh11_init(vusb11_port_t port);
void            vusbh11_shutdown(void);

/*
 * Main-loop pump. Call as often as you can (every iteration) to drive
 * deferred work the ISR can't safely do — currently: bus reset and
 * speed detection on the first ATTACH. Future steps (BDT setup, control
 * transfers, enumeration) will hang off this same call.
 */
void vusbh11_poll(void);

/* Read live register snapshot (call from main loop, NOT the ISR). */
typedef struct {
    uint32_t istat;
    uint32_t inten;
    uint32_t errstat;
    uint32_t ctl;
    uint32_t addr;
    uint32_t otgstat;
    uint32_t otgctl;
    uint32_t reg10;
    uint32_t frmnum;
    uint32_t stat;
} vusbh11_snapshot_t;

void vusbh11_snapshot(vusbh11_snapshot_t *out);

/* Set by the ISR on each TOKDNE so the main loop can dump the result.
 * `last_stat` is the STAT register snapshot (EP/DIR/PONG of the completed
 * transfer); `last_errstat` is whatever error bits accompanied it. */
extern volatile uint32_t vusbh11_last_tokdne_stat;
extern volatile uint32_t vusbh11_last_tokdne_errstat;

/* Post-completion peek into the BDT entry the controller wrote back to,
 * for diagnostics. Updated by the bottom-half, NOT the ISR. */
extern volatile uint32_t vusbh11_bdt_ep0_txeven_w0;  /* SETUP-out slot   */
extern volatile uint32_t vusbh11_bdt_ep0_rxodd_w0;   /* IN-data slot     */

/* The ISR (registered automatically by vusbh11_init). Exposed only so
 * tests can call it directly if needed. */
void vusbh11_isr(void);

/* ---- Public disk I/O (populated after process_attach succeeds) ---- */

/* Drive geometry. Both zero until enumeration + READ_CAPACITY succeed.
 * Callers can poll these as a "device ready?" check. */
extern volatile uint32_t vusbh11_block_count;
extern volatile uint32_t vusbh11_block_size;

/* Block-aligned SCSI read/write against a USB Mass Storage device that's
 * already been enumerated by the bottom-half (process_attach). Both work
 * in 512-byte sectors and chunk internally to fit the bulk DMA buffer.
 * Return 0 on success, -1 on failure. */
int vusbh11_disk_read (uint32_t lba, uint32_t n_blocks, void *dst);
int vusbh11_disk_write(uint32_t lba, uint32_t n_blocks, const void *src);

#endif
