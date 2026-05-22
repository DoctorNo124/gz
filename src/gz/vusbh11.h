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

typedef enum {
    VUSBH11_SPEED_UNKNOWN = 0,
    VUSBH11_SPEED_LOW,
    VUSBH11_SPEED_FULL,
} vusbh11_speed_t;

typedef enum {
    VUSBH11_ENUM_IDLE = 0,
    VUSBH11_ENUM_SETUP_SENT,
    VUSBH11_ENUM_IN_SENT,
    VUSBH11_ENUM_DONE,
    VUSBH11_ENUM_ERROR,
} vusbh11_enum_state_t;

/* ----- Pinned telemetry struct (placed at fixed VMA via gz.ld) -----
 *
 * All ISR/bottom-half observables live in this one struct so the
 * memory-viewer address stays stable across rebuilds. Linker script
 * `.usb_telem` section pins it at 0x803F0000 (between OoT runtime
 * and gz's load address — a safe gap).
 *
 * Field offsets are FROZEN by struct ordering; do not reorder unless
 * you update the in-game watchlist. Hex offsets are listed below for
 * use with gz's memory viewer.
 *
 * Offset  Field                  Size  Note
 *  +0x00  isr_count              u32   Total IRQs serviced
 *  +0x04  attach_count           u32   ATTACH events seen
 *  +0x08  rst_count              u32   USB bus-reset IRQs (peer-driven)
 *  +0x0C  tokdne_count           u32   Token transactions completed
 *  +0x10  softok_count           u32   SOFTOK ticks
 *  +0x14  error_count            u32   Controller error events
 *  +0x18  last_istat             u32   Most recent ISTAT snapshot
 *  +0x1C  last_errstat           u32   Most recent ERRSTAT snapshot
 *  +0x20  last_otgstat           u32   Most recent OTGSTAT
 *  +0x24  last_reg10             u32   Most recent wrapper IRQ status
 *  +0x28  last_tokdne_stat       u32   STAT (EP/dir/pong) at last TOKDNE
 *  +0x2C  last_tokdne_errstat    u32   ERRSTAT at last TOKDNE
 *  +0x30  bdt_ep0_txeven_w0      u32   BDT writeback — SETUP slot
 *  +0x34  bdt_ep0_rxodd_w0       u32   BDT writeback — IN-data slot
 *  +0x38  phase                  u32   Bottom-half progress (see below)
 *  +0x3C  block_count            u32   Disk sectors (post-READ_CAPACITY)
 *  +0x40  block_size             u32   Disk sector size in bytes
 *  +0x44  read_cap_last_lba      u32   Raw last-LBA from READ_CAPACITY
 *  +0x48  read_cap_blocksize     u32   Raw blocksize from READ_CAPACITY
 *  +0x4C  device_speed           u8    0=unknown 1=LOW 2=FULL
 *  +0x4D  enum_state             u8    Chapter-9 enum stage
 *  +0x4E  dev_desc_valid         u8    Got GET_DESC(DEVICE,8)?
 *  +0x4F  attach_pending         u8    ISR→bottom-half flag
 *  +0x50  inquiry_status         u8    0xEE not run, 0x00 ok, 0xFE failed
 *  +0x51  tur_status             u8    SCSI status (0=ready, non-zero=not)
 *  +0x52  read_cap_status        u8    SCSI status
 *  +0x53  (pad)                  u8
 *  +0x54  read_cap_got           u16   Bytes received in data phase
 *  +0x56  (pad)                  u16
 *  +0x58  dev_desc[8]            u8[8] First 8 bytes of device descriptor
 *
 * `phase` meanings:
 *   0  = never entered process_attach
 *   1  = entered, about to drive USB reset
 *   2  = post-reset, speed detected
 *   3  = first SETUP issued + ACK
 *   4  = first IN data received
 *   5  = first STATUS OUT ZLP done
 *   6  = SET_ADDRESS(1) done
 *   7  = GET_DESC(DEVICE,18) done
 *   8  = GET_DESC(CONFIG,9) done
 *   9  = GET_DESC(CONFIG,full) done
 *  10  = SET_CONFIGURATION(1) done
 *  11  = INQUIRY done
 *  12  = TUR done
 *  13  = REQUEST_SENSE done
 *  14  = READ_CAPACITY done
 *  15  = block geometry published — success
 */
typedef struct {
    uint32_t isr_count;
    uint32_t attach_count;
    uint32_t rst_count;
    uint32_t tokdne_count;
    uint32_t softok_count;
    uint32_t error_count;
    uint32_t last_istat;
    uint32_t last_errstat;
    uint32_t last_otgstat;
    uint32_t last_reg10;
    uint32_t last_tokdne_stat;
    uint32_t last_tokdne_errstat;
    uint32_t bdt_ep0_txeven_w0;
    uint32_t bdt_ep0_rxodd_w0;
    uint32_t phase;
    uint32_t block_count;
    uint32_t block_size;
    uint32_t read_cap_last_lba;
    uint32_t read_cap_blocksize;
    uint8_t  device_speed;        /* vusbh11_speed_t */
    uint8_t  enum_state;          /* vusbh11_enum_state_t */
    uint8_t  dev_desc_valid;
    uint8_t  attach_pending;
    uint8_t  inquiry_status;
    uint8_t  tur_status;
    uint8_t  read_cap_status;
    uint8_t  _pad0;
    uint16_t read_cap_got;
    uint16_t _pad1;
    uint8_t  dev_desc[8];
    /* Diagnostics for the first SETUP transaction. last_setup_w0 is the
     * BDT word-0 the controller wrote back after our GET_DESCRIPTOR
     * SETUP — the PID (bits 5:2 of byte 0) tells us why it failed:
     *   0x2 = ACK     (good)
     *   0x0 = BUSTIMEOUT  (device didn't respond — BTOERR)
     *   0xA = NAK
     *   0xE = STALL */
    uint32_t last_setup_w0;
    uint32_t first_setup_retries;
} vusbh11_telem_t;

#define VUSBH11_TELEM_ADDR  0x803F0000u

extern volatile vusbh11_telem_t vusbh11_telem;

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

/* The ISR (registered automatically by vusbh11_init). Exposed only so
 * tests can call it directly if needed. */
void vusbh11_isr(void);

/* ---- Public disk I/O (block_size/count are inside vusbh11_telem) ----
 *
 * Block-aligned SCSI read/write against a USB Mass Storage device that's
 * already been enumerated by the bottom-half (process_attach). Both work
 * in 512-byte sectors and chunk internally to fit the bulk DMA buffer.
 * Return 0 on success, -1 on failure. Caller can check
 * vusbh11_telem.block_size != 0 to test "device ready". */
int vusbh11_disk_read (uint32_t lba, uint32_t n_blocks, void *dst);
int vusbh11_disk_write(uint32_t lba, uint32_t n_blocks, const void *src);

#endif
