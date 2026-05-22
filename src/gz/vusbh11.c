#include "z64.h"
#if Z64_VERSION == Z64_OOTIQC

#ifndef USB_HAL_BACKEND_GZ
#  define USB_HAL_BACKEND_GZ
#endif
#include <stdbool.h>
#include <stdint.h>
#include "usb_hal.h"
#include "vusb11.h"
#include "vusbh11.h"

/*
 * VUSBH11 — host-mode bring-up.
 *
 * Sequence ported from Thar0's libultra ique-usb fork:
 *   __usbCtlrInit(which)  →  __usbOtgStateChange(which)  →  __usbHostMode(which)
 *
 * __usbCtlrInit (usbhw.c):
 *   IO_WRITE(MI_3C_REG, (which == 0) ? (1 << 20) : (1 << 22));
 *   IO_WRITE(USB_CTL_REG(which),     USB_CTL_USB_DISABLE);
 *   IO_WRITE(USB_OTGICR_REG(which),  USB_OTGICR_NONE);
 *   IO_WRITE(USB_ERREN_REG(which),   USB_ERREN_NONE);
 *   IO_WRITE(USB_INTEN_REG(which),   USB_INTEN_NONE);
 *   IO_WRITE(USB_OTGISTAT_REG(which),USB_OTGISTAT_ALL);
 *   IO_WRITE(USB_ERRSTAT_REG(which), USB_ERRSTAT_ALL);
 *   IO_WRITE(USB_ISTAT_REG(which),   USB_ISTAT_ALL);
 *   IO_WRITE(USB_CTL_REG(which),     USB_CTL_ODD_RST | USB_CTL_USB_DISABLE);
 *   IO_WRITE(USB_ADDR_REG(which),    USB_ADDR_FSEN | USB_ADDR_ADDR(0));
 *   IO_WRITE(USB_BDTPAGE1_REG(which), USB_BDTPAGE1_ADDR(addr));
 *   IO_WRITE(USB_BDTPAGE2_REG(which), USB_BDTPAGE2_ADDR(addr));
 *   IO_WRITE(USB_BDTPAGE3_REG(which), USB_BDTPAGE3_ADDR(addr));
 *
 * __usbHostMode (usbhw.c):
 *   IO_WRITE(USB_OTGCTL_REG_ALT(which),
 *            USB_OTGCTL_DPLOW | USB_OTGCTL_DMLOW |
 *            USB_OTGCTL_RESERVED3 | USB_OTGCTL_OTGEN);                  // 0x3C
 *   __usb_arc_host_setup(which, &__osArcHostHandle[which]);
 *   IO_WRITE(MI_3C_REG, (which == 0) ? (1 << 21) : (1 << 23));
 *
 * __usb_arc_host_setup → _usb_host_init → _usb_hci_vusb11_init (vusbh11ma.c):
 *   ENDPT_HOST_RG[0] = 0;
 *   CONTROL = USB_CTL_USB_DISABLE;
 *   INTENABLE = USB_INTEN_NONE;
 *   INTSTATUS = USB_ISTAT_ALL;
 *   CONTROL = USB_CTL_ODD_RST | USB_CTL_USB_DISABLE;
 *   ADDRESS = USB_ADDR_FSEN | USB_ADDR_ADDR(0);
 *   (set BDTPAGE1/2/3 again)
 *   CONTROL = USB_CTL_HOST_MODE_EN | USB_CTL_USB_DISABLE;       // 0x08
 *   SOFTHRESHOLDLO = USB_SOF_THRESHOLD(64);                     // 10+64 = 74
 *   INTENABLE = USB_INTEN_ATTACHEN;                             // 0x40
 *
 * After that, ATTACH fires when a device pulls D+ high.
 *
 * NOTE on threading: HANDOFF.md warns that doing real work in the ISR will
 * miss SOFs. For now the ISR just snapshots state, counts events, clears
 * the W1C bits, and returns. Bottom-half work (process_attach → bus reset
 * → speed detect → GET_DESCRIPTOR) belongs in a polled state machine that
 * we'll layer on later.
 */

/* ----- State ----- */

static vusb11_port_t s_port    = VUSB11_PORT_USB0;
static bool          s_initted = false;

/* Current target device address. Thar0's send_token always re-writes the
 * ADDRESS register before each token, so we mirror that pattern from a
 * tracked value. Set to 0 at boot, 1 after SET_ADDRESS succeeds. */
static uint8_t s_current_addr = 0;

/* Forward declarations of the toggle arrays so process_attach can poke
 * them directly when a CLEAR_FEATURE resets the device-side toggles.
 * (Defined below in the bulk-transfer state section.) */
static uint8_t s_ep_out_toggle[16];
static uint8_t s_ep_in_toggle[16];
static uint8_t s_rx_next_pong;
static uint8_t s_tx_next_pong;

/* ISR-driven transaction completion state (one slot per direction —
 * we only ever have one token in flight at a time).
 *
 * The ISR processes TOKDNE end-to-end: reads STAT for direction, reads
 * the BDT slot at our current SW pong, advances pong, saves the w0
 * writeback into s_*_last_w0, sets s_*_tokdne_pending = 1. App-side
 * waits for the pending flag via wait_for_completion(). This matches
 * iquesync/Thar0's architecture — the ISR fires only AFTER the
 * controller is fully done (TXSUSPENDTOKENBUSY cleared, internal pong
 * selector advanced), so there's no race with us issuing the next
 * token. SW pong stays in lockstep because each TOKDNE = one ^=1, both
 * in HW and in the ISR. */
static volatile uint8_t  s_rx_tokdne_pending = 0;
static volatile uint8_t  s_tx_tokdne_pending = 0;
static volatile uint32_t s_rx_last_w0 = 0;
static volatile uint32_t s_tx_last_w0 = 0;

volatile uint32_t vusbh11_isr_count    = 0;
volatile uint32_t vusbh11_attach_count = 0;
volatile uint32_t vusbh11_rst_count    = 0;
volatile uint32_t vusbh11_tokdne_count = 0;
volatile uint32_t vusbh11_softok_count = 0;
volatile uint32_t vusbh11_error_count  = 0;
volatile uint32_t vusbh11_last_istat   = 0;
volatile uint32_t vusbh11_last_errstat = 0;
volatile uint32_t vusbh11_last_otgstat = 0;
volatile uint32_t vusbh11_last_reg10   = 0;
volatile bool     vusbh11_attach_pending = false;
volatile vusbh11_speed_t vusbh11_device_speed = VUSBH11_SPEED_UNKNOWN;
volatile uint32_t vusbh11_last_tokdne_stat    = 0;
volatile uint32_t vusbh11_last_tokdne_errstat = 0;
volatile uint32_t vusbh11_bdt_ep0_txeven_w0   = 0;
volatile uint32_t vusbh11_bdt_ep0_rxodd_w0    = 0;
volatile uint8_t  vusbh11_dev_desc[8]         = {0};
volatile bool     vusbh11_dev_desc_valid      = false;
volatile vusbh11_enum_state_t vusbh11_enum_state = VUSBH11_ENUM_IDLE;

/* Storage geometry — populated by process_attach after a successful
 * READ_CAPACITY. vusbh11_disk_read/write read these. Zero means
 * "not ready yet" (no device attached, or enumeration failed). */
volatile uint32_t vusbh11_block_count = 0;
volatile uint32_t vusbh11_block_size  = 0;

/* ----- BDT in controller MMIO (+0x80000 region) -----
 *
 * Re-thought: the previous RAM-backed BDT actually completed the SETUP TX
 * transaction (BDT write-back happened, OWN cleared), so DMA can reach
 * RDRAM. But the IN data phase stalled — TX_SUSPEND_BUSY locked on, no
 * TOKDNE, no ERRSTAT. The strongest hypothesis is that the controller's
 * RX/receive-writeback path on this chip is sensitive to BDT memory
 * placement in a way the TX path isn't, and the controller-local MMIO at
 * +0x80000 is the only region it fully supports for receive.
 *
 * iquesync's working device-mode driver uses controller MMIO for its BDT
 * (its `bdt_base = ctrl_base + 0x80000`), and that's been smoke-tested.
 * Mirroring it here.
 *
 * BDT base in CPU-visible KSEG1 (uncached MMIO): controller_base + 0x80000
 *   USB0 → 0xA4980000     physical → 0x04980000
 *   USB1 → 0xA4A80000     physical → 0x04A80000
 *
 * Layout within the BDT region:
 *   0x00..0x1F  EP0 BDT entries (RX Even/Odd, TX Even/Odd)
 *   0x40..0x47  SETUP packet outbound buffer
 *   0x60..0x67  IN response buffer
 */

/* BDT slot indexing (offset = (EP*4 + DIR*2 + PONG) * 8). DIR: 0=RX, 1=TX. */
#define BDT_EP0_RX_EVEN_OFF  0x00u
#define BDT_EP0_RX_ODD_OFF   0x08u
#define BDT_EP0_TX_EVEN_OFF  0x10u
#define BDT_EP0_TX_ODD_OFF   0x18u
#define BDT_EP1_RX_EVEN_OFF  0x20u   /* bulk IN data + CSW              */
#define BDT_EP1_RX_ODD_OFF   0x28u
#define BDT_EP2_TX_EVEN_OFF  0x50u   /* bulk OUT (CBW + data-OUT)       */
#define BDT_EP2_TX_ODD_OFF   0x58u

/* Buffers live in regular RDRAM, NOT the controller MMIO BDT region.
 * The MMIO region has BDT entry slots that the controller may write
 * during transactions (at fixed offsets per the (EP*4+DIR*2+PONG)*8
 * formula), and buffers placed there can be silently corrupted if any
 * BDT writeback overlaps. RDRAM has no such interactions.
 *
 * Layout within s_dma_buf:
 *   0x0000..0x0007  SETUP packet  (8 bytes used)
 *   0x0040..0x007F  control-IN response  (64 bytes)
 *   0x0080..0x00BF  CBW  (31 bytes used, padded to 64)
 *   0x00C0..0x00FF  CSW  (13 bytes used, padded to 64)
 *   0x0100..0x10FF  bulk data IN/OUT  (4096 bytes = 8 sectors, shared) */
#define BUF_SETUP_OFF        0x0000u
#define BUF_RX_OFF           0x0040u
#define BUF_CBW_OFF          0x0080u
#define BUF_CSW_OFF          0x00C0u
#define BUF_BULK_IN_OFF      0x0100u
#define BUF_BULK_OUT_OFF     0x0100u  /* same buffer — never used concurrently */

static uint32_t bdt_uncached_base(void)
{
    uint32_t base = (s_port == VUSB11_PORT_USB0)
                  ? VUSB11_USB0_BASE : VUSB11_USB1_BASE;
    return base + VUSB11_OFF_BDT_MEM;
}
static uint32_t bdt_phys_base(void)
{
    return (s_port == VUSB11_PORT_USB0)
         ? VUSB11_USB0_BDT_PHYS
         : VUSB11_USB1_BDT_PHYS;
}

/* Data buffers live in RDRAM (not controller MMIO). The controller can
 * DMA to/from regular RAM by physical address; BDT word1 just needs to
 * carry the physical address. CPU access via the KSEG1 uncached mirror
 * keeps it coherent without explicit cache flushes. */
static uint8_t s_dma_buf[8192] __attribute__((aligned(32)));

static uint32_t dma_uncached_base(void)
{
    return (((uint32_t)s_dma_buf & 0x1FFFFFFFu) | 0xA0000000u);
}
static uint32_t dma_phys_base(void)
{
    return ((uint32_t)s_dma_buf & 0x1FFFFFFFu);
}

static uint32_t swab32(uint32_t x)
{
    return ((x & 0xFF000000u) >> 24) |
           ((x & 0x00FF0000u) >>  8) |
           ((x & 0x0000FF00u) <<  8) |
           ((x & 0x000000FFu) << 24);
}

/* Forward declarations for helpers used by process_attach but defined
 * below (kept low in the file so they're near vusbh11_poll). */
static bool     wait_token_busy_clear(void);
static void     issue_token(uint8_t token_byte);
static bool     send_inquiry(void);
static uint8_t  send_bot_in_cmd(const uint8_t *cdb, uint8_t cdb_len,
                                uint32_t data_buf_off, uint32_t expected,
                                uint16_t *received_out);
static uint8_t  send_bot_out_cmd(const uint8_t *cdb, uint8_t cdb_len,
                                 uint32_t data_buf_off, uint32_t length,
                                 uint16_t *sent_out);
static void     read_buf_bytes(uint8_t *dst, uint32_t buf_off, uint16_t n);
static void     write_bytes_to_buf(uint32_t buf_off, const uint8_t *src, uint16_t n);

/* Write a BDT word0 from CTRL byte + 10-bit BC.
 *
 * Memory layout (BE-mapped 32-bit word):
 *   byte 0  (bits 31:24) = CTRL    (OWN/DATA01/KEEP/NINC/DTS/...)
 *   byte 1  (bits 23:16) = reserved
 *   byte 2  (bits 15: 8) = BC[ 7:0]
 *   byte 3  (bits  7: 0) = BC[9:8]
 *
 * For BC=8: 0x88000800 (CTRL=0x88, BC[7:0]=0x08 at byte 2). */
static uint32_t make_bdt_w0(uint8_t ctrl, uint16_t bc)
{
    return ((uint32_t)ctrl << 24)
         | (((uint32_t)(bc & 0xFFu)) << 8)
         |  (((uint32_t)bc >> 8) & 0x03u);
}

/* Prime a BDT slot with the given CTRL byte, byte count and buffer phys. */
static void prime_bdt_slot(uint32_t bdt_off, uint8_t ctrl, uint16_t bc, uint32_t buf_phys)
{
    uint32_t bdt = bdt_uncached_base();
    usb_hal_io_write(bdt + bdt_off + 4u, swab32(buf_phys));
    usb_hal_io_write(bdt + bdt_off + 0u, make_bdt_w0(ctrl, bc));
}

/* Wait for EITHER pong (Even or Odd) of a direction to clear OWN.
 * Returns w0 of the slot that completed; *which_off is set to the offset.
 * Returns 0 on timeout (with a dump of relevant registers). */
static void trace(const char *step)
{
    usb_hal_log("vusbh11: -> %s\n", step);
    usb_hal_log("vusbh11 -> %s\n", step);
}

static uint32_t bdt_phys(vusb11_port_t port)
{
    return (port == VUSB11_PORT_USB0) ? VUSB11_USB0_BDT_PHYS
                                      : VUSB11_USB1_BDT_PHYS;
}

static uint32_t mi_init_bit(vusb11_port_t port)
{
    return (port == VUSB11_PORT_USB0) ? MI_USB0_CTLRINIT_BIT
                                      : MI_USB1_CTLRINIT_BIT;
}

static uint32_t mi_mode_bit(vusb11_port_t port)
{
    return (port == VUSB11_PORT_USB0) ? MI_USB0_MODE_BIT
                                      : MI_USB1_MODE_BIT;
}

/* ----- __usbCtlrInit equivalent ----- */

static void ctlr_init(vusb11_port_t port)
{
    uint32_t bdt = bdt_phys(port);

    /* Gate the per-port MI bit for the ctlr_init phase (per SA1, sw not OR). */
    usb_hal_io_write(MI_HW_INTR_MASK_REG, mi_init_bit(port));

    /* Quiesce. CTL=0 means USB disabled. */
    vusb11_write(port, VUSB11_OFF_CTL,     0u);
    vusb11_write(port, VUSB11_OFF_ERREN,   0u);
    vusb11_write(port, VUSB11_OFF_INTEN,   0u);

    /* Clear all W1C status. */
    vusb11_write(port, VUSB11_OFF_ERRSTAT, 0xFFu);
    vusb11_write(port, VUSB11_OFF_ISTAT,   0xFFu);

    /* ODD_RST resets the controller's BDT EVEN/ODD bank tracking.
     * (Thar0: USB_CTL_ODD_RST | USB_CTL_USB_DISABLE) */
    vusb11_write(port, VUSB11_OFF_CTL, VUSB11_CTL_ODD_RST);

    /* Address = 0, full-speed. */
    vusb11_write(port, VUSB11_OFF_ADDR, VUSB11_ADDR_FSEN | 0u);

    /* Split-write BDT physical address into BDTPAGE1/2/3. The hardware
     * concatenates: BDT_base = (BDTPAGE3 << 24) | (BDTPAGE2 << 16) | (BDTPAGE1 << 8).
     * Bits [7:0] are implicitly zero (BDT is 256-byte aligned). */
    vusb11_write(port, VUSB11_OFF_BDTPAGE1, (bdt >> 8)  & 0xFFu);
    vusb11_write(port, VUSB11_OFF_BDTPAGE2, (bdt >> 16) & 0xFFu);
    vusb11_write(port, VUSB11_OFF_BDTPAGE3, (bdt >> 24) & 0xFFu);
}

/* ----- _usb_hci_vusb11_init equivalent (host-side controller setup) ----- */

static void hci_init(vusb11_port_t port)
{
    uint32_t bdt = bdt_phys(port);

    /* Clear endpoint 0 control. */
    vusb11_write(port, VUSB11_OFF_EP_CTL_BASE + 0u * 4u, 0u);

    /* Re-quiesce + re-program BDTPAGE (Thar0's host init does this AFTER
     * __usbCtlrInit, redundant on paper but mirrored anyway). */
    vusb11_write(port, VUSB11_OFF_CTL,     0u);
    vusb11_write(port, VUSB11_OFF_INTEN,   0u);
    vusb11_write(port, VUSB11_OFF_ISTAT,   0xFFu);
    vusb11_write(port, VUSB11_OFF_CTL,     VUSB11_CTL_ODD_RST);
    vusb11_write(port, VUSB11_OFF_ADDR,    VUSB11_ADDR_FSEN | 0u);

    vusb11_write(port, VUSB11_OFF_BDTPAGE1, (bdt >> 8)  & 0xFFu);
    vusb11_write(port, VUSB11_OFF_BDTPAGE2, (bdt >> 16) & 0xFFu);
    vusb11_write(port, VUSB11_OFF_BDTPAGE3, (bdt >> 24) & 0xFFu);

    /* Flip into host mode. USB_EN stays OFF here — _usb_host_vusb11_process_attach
     * is what eventually sets USB_EN once a bus reset has been driven. */
    vusb11_write(port, VUSB11_OFF_CTL, VUSB11_CTL_HOST_MODE_EN);

    /* SOF threshold = 10 + max_packet_size (full-speed = 64). */
    vusb11_write(port, VUSB11_OFF_SOFTHLD, 10u + 64u);

    /* Enable just the ATTACH interrupt. Everything else stays masked
     * until process_attach has detected speed and reset the bus. */
    vusb11_write(port, VUSB11_OFF_INTEN, VUSB11_INTEN_ATTACHEN);
}

/* ----- __usbHostMode equivalent (OTG wrapper + MI gate) ----- */

static void host_mode(vusb11_port_t port)
{
    /* OTGCTL = 0x3C: D+/D- pull-downs (host signals to device), iQue's
     * RESERVED3 bit (likely VBUS supply enable on iQue's PHY mux), and
     * OTGEN to actually let the OTG block run. */
    vusb11_write(port, VUSB11_OFF_OTGCTL, VUSB11_OTGCTL_HOST_VALUE);

    /* Host controller state setup (the work _usb_host_init/__usb_arc_host_setup
     * does for Thar0 — BDT mapping, endpoint 0, ATTACH-only mask). */
    hci_init(port);

    /* Re-arm the per-port MI gate at the "mode-select phase" value (per SA1,
     * sw not OR — this overwrites the ctlr_init bit). */
    usb_hal_io_write(MI_HW_INTR_MASK_REG, mi_mode_bit(port));
}

/* ----- Public entry ----- */

vusb11_status_t vusbh11_init(vusb11_port_t port)
{
    s_port = port;

    /* Register handlers BEFORE making the controller capable of firing
     * interrupts — libdragon's MI dispatcher asserts if a USB IRQ fires
     * with no handler. We register on BOTH ports defensively; only the
     * one we initialize will actually fire ATTACH/etc. */
    trace("entered vusbh11_init");
    trace("register BB_USB0 + BB_USB1 handlers");
    usb_hal_irq_install(0, vusbh11_isr);
    usb_hal_irq_install(1, vusbh11_isr);

    trace("ctlr_init(host port)");
    ctlr_init(port);

    /* The other port still needs to be brought into SOME consistent state
     * so its wrapper doesn't drive spurious signals — quiesce it. */
    {
        vusb11_port_t other = (port == VUSB11_PORT_USB0) ? VUSB11_PORT_USB1
                                                         : VUSB11_PORT_USB0;
        trace("ctlr_init(other port — quiesce)");
        ctlr_init(other);
    }

    trace("host_mode(host port)");
    host_mode(port);

    trace("enable BB_USB0 + BB_USB1 IRQ lines at MI");
    usb_hal_irq_enable(0, true);
    usb_hal_irq_enable(1, true);

    s_initted = true;
    trace("vusbh11_init done — waiting for ATTACH");
    return VUSB11_OK;
}

void vusbh11_shutdown(void)
{
    if (!s_initted) return;
    usb_hal_irq_enable(0, false);
    usb_hal_irq_enable(1, false);
    usb_hal_irq_remove(0);
    usb_hal_irq_remove(1);

    /* Park the host port back to "USB disabled, ATTACH masked". */
    vusb11_write(s_port, VUSB11_OFF_INTEN, 0u);
    vusb11_write(s_port, VUSB11_OFF_CTL,   0u);
    s_initted = false;
}

/* ----- Reusable control-transfer helpers ----- */

/* Write 8 bytes into the SETUP buffer. Bytes are placed in physical order
 * (byte 0 of `pkt` goes to lowest memory address, then byte 1, etc.) so
 * the on-wire ordering matches `pkt` exactly. */
static void write_setup_packet(const uint8_t pkt[8])
{
    uint32_t setup = dma_uncached_base() + BUF_SETUP_OFF;
    uint32_t w0 = ((uint32_t)pkt[0] << 24) | ((uint32_t)pkt[1] << 16) |
                  ((uint32_t)pkt[2] <<  8) |  (uint32_t)pkt[3];
    uint32_t w1 = ((uint32_t)pkt[4] << 24) | ((uint32_t)pkt[5] << 16) |
                  ((uint32_t)pkt[6] <<  8) |  (uint32_t)pkt[7];
    usb_hal_io_write(setup + 0, w0);
    usb_hal_io_write(setup + 4, w1);
}

/* Wait for the ISR to flag a completed transaction in the requested
 * direction, then return the w0 the ISR captured. Returns 0 on timeout.
 *
 * `is_host_rx` = true for IN tokens (host receives), false for OUT/SETUP
 * (host transmits). The ISR routes the completion to s_rx_* or s_tx_*
 * based on STAT's IN bit, so this just polls the right flag. */
static uint32_t wait_for_completion(bool is_host_rx)
{
    volatile uint8_t *flag = is_host_rx ? &s_rx_tokdne_pending : &s_tx_tokdne_pending;
    int spins = 0;
    while (*flag == 0u) {
        if (++spins > 5000000) {
            usb_hal_log("vusbh11: wait_for_completion(%s) TIMEOUT\n",
                   is_host_rx ? "RX" : "TX");
            usb_hal_log("  CTL=%02lx ISTAT=%02lx INTEN=%02lx ERRSTAT=%02lx STAT=%02lx\n",
                   (unsigned long)(vusb11_read(s_port, VUSB11_OFF_CTL)     & 0xFFu),
                   (unsigned long)(vusb11_read(s_port, VUSB11_OFF_ISTAT)   & 0xFFu),
                   (unsigned long)(vusb11_read(s_port, VUSB11_OFF_INTEN)   & 0xFFu),
                   (unsigned long)(vusb11_read(s_port, VUSB11_OFF_ERRSTAT) & 0xFFu),
                   (unsigned long)(vusb11_read(s_port, VUSB11_OFF_STAT)    & 0xFFu));
            return 0u;
        }
    }
    *flag = 0;
    return is_host_rx ? s_rx_last_w0 : s_tx_last_w0;
}

/* All control transfer functions use SINGLE-PONG + DTS=1 + hard-coded
 * DATA01 per BOT/USB spec, matching Thar0's discipline. Each call primes
 * the slot indicated by s_*_next_pong; the ISR advances the tracker on
 * each TOKDNE so SW and HW stay in lockstep. */

static bool do_setup_phase(uint8_t addr_for_log)
{
    uint32_t setup_phys = dma_phys_base() + BUF_SETUP_OFF;
    uint32_t off = s_tx_next_pong ? BDT_EP0_TX_ODD_OFF : BDT_EP0_TX_EVEN_OFF;
    /* SETUP is always DATA0 per USB spec. CTRL=0x88 = OWN|DTS, no DATA01. */
    prime_bdt_slot(off, 0x88, 8, setup_phys);
    issue_token(VUSB11_TOKEN_SETUP(0));

    uint32_t w0 = wait_for_completion(false);  /* SETUP is host TX */
    if (w0 == 0u) {
        usb_hal_log("vusbh11: SETUP[addr=%u] timed out\n", addr_for_log);
        return false;
    }
    uint8_t pid = (uint8_t)(((w0 >> 24) >> 2) & 0xFu);
    if (pid != 0x2u) {
        usb_hal_log("vusbh11: SETUP[addr=%u] not ACK'd, pid=%x\n", addr_for_log, pid);
        return false;
    }
    return true;
}

/* IN data phase (control). For our use case all responses fit in one
 * mps=64-byte packet, so this is single-packet. USB spec: first IN
 * packet of the data phase is DATA1. CTRL=0xC8 = OWN|DTS|DATA1, so
 * the controller rejects any packet whose wire PID doesn't match —
 * forcing the device to retransmit with the correct toggle and keeping
 * us in lockstep. Matches Thar0's vusbh11ma.c host-mode discipline. */
static uint16_t do_in_data_phase(uint16_t requested)
{
    uint32_t rx_phys = dma_phys_base() + BUF_RX_OFF;
    for (int attempt = 0; attempt < 50; attempt++) {
        uint32_t off = s_rx_next_pong ? BDT_EP0_RX_ODD_OFF : BDT_EP0_RX_EVEN_OFF;
        prime_bdt_slot(off, 0xC8, 64, rx_phys);   /* OWN|DTS|DATA1 */
        issue_token(VUSB11_TOKEN_IN(0));

        uint32_t w0 = wait_for_completion(true);  /* IN is host RX */
        if (w0 == 0u) {
            usb_hal_log("vusbh11: IN attempt %d timed out\n", attempt);
            return 0;
        }
        uint8_t pid = (uint8_t)(((w0 >> 24) >> 2) & 0xFu);
        if (pid == 0xAu) { usb_hal_wait_ms(10); continue; }   /* NAK, retry */
        if (pid == 0xEu) { usb_hal_log("vusbh11: IN STALL\n"); return 0; }
        if (pid != 0x3u && pid != 0xBu) {
            usb_hal_log("vusbh11: IN unexpected pid=%x\n", pid);
            return 0;
        }
        (void)requested;
        return (uint16_t)(((w0 >> 8) & 0xFFu) | ((w0 & 0x03u) << 8));
    }
    usb_hal_log("vusbh11: IN gave up after 50 NAKs\n");
    return 0;
}

/* STATUS OUT ZLP — always DATA1 per USB spec. */
static bool do_status_out_zlp(void)
{
    uint32_t setup_phys = dma_phys_base() + BUF_SETUP_OFF;
    uint32_t off = s_tx_next_pong ? BDT_EP0_TX_ODD_OFF : BDT_EP0_TX_EVEN_OFF;
    prime_bdt_slot(off, 0xC8, 0, setup_phys);   /* OWN|DTS|DATA1, BC=0 */
    issue_token(VUSB11_TOKEN_OUT(0));

    uint32_t w0 = wait_for_completion(false);   /* OUT is host TX */
    if (w0 == 0u) {
        usb_hal_log("vusbh11: STATUS OUT timed out\n");
        return false;
    }
    uint8_t pid = (uint8_t)(((w0 >> 24) >> 2) & 0xFu);
    if (pid != 0x2u) {
        usb_hal_log("vusbh11: STATUS OUT not ACK'd, pid=%x\n", pid);
        return false;
    }
    return true;
}

/* STATUS IN ZLP — USB spec: always DATA1. CTRL=0xC8 enforces it. */
static bool do_status_in_zlp(void)
{
    uint32_t rx_phys = dma_phys_base() + BUF_RX_OFF;
    for (int attempt = 0; attempt < 20; attempt++) {
        uint32_t off = s_rx_next_pong ? BDT_EP0_RX_ODD_OFF : BDT_EP0_RX_EVEN_OFF;
        prime_bdt_slot(off, 0xC8, 0, rx_phys);   /* OWN|DTS|DATA1, BC=0 */
        issue_token(VUSB11_TOKEN_IN(0));

        uint32_t w0 = wait_for_completion(true);
        if (w0 == 0u) { usb_hal_log("vusbh11: STATUS IN timed out\n"); return false; }
        uint8_t pid = (uint8_t)(((w0 >> 24) >> 2) & 0xFu);
        if (pid == 0xAu) { usb_hal_wait_ms(10); continue; }
        if (pid == 0xBu || pid == 0x3u) return true;
        usb_hal_log("vusbh11: STATUS IN unexpected pid=%x\n", pid);
        return false;
    }
    usb_hal_log("vusbh11: STATUS IN gave up\n");
    return false;
}

/* Read N bytes from the RX buffer into dst[]. */
static void read_rx_bytes(uint8_t *dst, uint16_t n)
{
    uint32_t rx = dma_uncached_base() + BUF_RX_OFF;
    for (uint16_t i = 0; i < n; i++) {
        uint32_t w = usb_hal_io_read(rx + (i & ~3u));
        dst[i] = (uint8_t)(w >> ((3 - (i & 3)) * 8));
    }
}

/* ----- Bottom-half: process the first attach ----- */

static void process_attach(void)
{
    usb_hal_log("vusbh11: process_attach -> driving USB reset\n");

    /* Hold the bus in reset (SE0) for ~10ms. USB spec minimum is 10ms; the
     * device's pull-up will reassert J state once we release. Keep
     * HOST_MODE_EN set throughout so the controller stays in host role. */
    vusb11_write(s_port, VUSB11_OFF_CTL,
                 VUSB11_CTL_RESET | VUSB11_CTL_HOST_MODE_EN);
    usb_hal_wait_ms(10);

    /* Release reset AND enable the USB module — this is what starts SOF
     * generation at 1kHz on the bus. Without USB_EN the controller never
     * emits frames and an attached device times out and re-resets itself. */
    vusb11_write(s_port, VUSB11_OFF_CTL,
                 VUSB11_CTL_HOST_MODE_EN | VUSB11_CTL_USB_EN);

    /* USB spec gives the device up to 10 ms to settle after reset (TRSTRCY),
     * and many devices need longer in practice. Wait 100 ms to be safe —
     * we're not in a hurry on the first enumeration. */
    usb_hal_wait_ms(100);

    /* Pulse ODD_RST to re-sync the BDT pong (Even/Odd) latch with our
     * fresh BDT entries. Without this, the controller's hardware pong
     * state can be misaligned after a bus reset and silently refuse the
     * RX BDT slot we primed. ODD_RST is a toggle, not sticky — set with
     * USB_EN and HOST_MODE_EN, then clear it again in the same write. */
    vusb11_write(s_port, VUSB11_OFF_CTL,
                 VUSB11_CTL_ODD_RST | VUSB11_CTL_HOST_MODE_EN | VUSB11_CTL_USB_EN);
    vusb11_write(s_port, VUSB11_OFF_CTL,
                 VUSB11_CTL_HOST_MODE_EN | VUSB11_CTL_USB_EN);

    /* Speed detect. After reset, the device's pull-up holds the bus in J
     * state. On a full-speed bus J = D+ high (CTL.JSTATE set). On a
     * low-speed bus the polarity is inverted — JSTATE clear, SE0 clear,
     * meaning D- is the one being held high. */
    uint32_t ctl = vusb11_read(s_port, VUSB11_OFF_CTL);
    if (ctl & VUSB11_CTL_JSTATE) {
        vusbh11_device_speed = VUSBH11_SPEED_FULL;
        usb_hal_log("vusbh11: device = FULL-speed (CTL=%02lx)\n",
               (unsigned long)(ctl & 0xFFu));
    } else if (!(ctl & VUSB11_CTL_SE0)) {
        vusbh11_device_speed = VUSBH11_SPEED_LOW;
        usb_hal_log("vusbh11: device = LOW-speed (CTL=%02lx)\n",
               (unsigned long)(ctl & 0xFFu));
    } else {
        usb_hal_log("vusbh11: bus still SE0 post-reset (CTL=%02lx) — device gone?\n",
               (unsigned long)(ctl & 0xFFu));
    }

    /* Clear leftover ISTAT/ERRSTAT — RST will have asserted while we held
     * the reset, and the wrapper REG10 mirrors it. */
    vusb11_write(s_port, VUSB11_OFF_ISTAT,   0xFFu);
    vusb11_write(s_port, VUSB11_OFF_ERRSTAT, 0xFFu);
    vusb11_write(s_port, VUSB11_OFF_REG10,   0xFFu);

    /* Enable error reporting and the next layer of interrupts.
     * Deliberately NOT enabling SOFTOKEN yet — at 1kHz it would dominate
     * the screen output. FRM in the snapshot will tell us SOFs are
     * happening at the hardware level. */
    vusb11_write(s_port, VUSB11_OFF_ERREN, 0xFFu);
    vusb11_write(s_port, VUSB11_OFF_INTEN,
                 VUSB11_INTEN_TOKDNEEN |
                 VUSB11_INTEN_USBRSTEN |
                 VUSB11_INTEN_ERROREN);

    usb_hal_log("vusbh11: process_attach done; bus running\n");
    usb_hal_log("vusbh11: BDT at controller MMIO phys 0x%08lx\n",
           (unsigned long)bdt_phys_base());

    /* Zero the BDT region (4 EP0 entries) so unused slots have OWN=0. */
    for (uint32_t off = 0; off < 0x20u; off += 4u) {
        usb_hal_io_write(bdt_uncached_base() + off, 0u);
    }

    /* Write the 8-byte SETUP packet for GET_DESCRIPTOR(DEVICE, 8).
     *   bmRequestType = 0x80 (IN, Standard, Device)
     *   bRequest      = 0x06 (GET_DESCRIPTOR)
     *   wValue        = 0x0100 (DEVICE descriptor, index 0)  [LE on wire]
     *   wIndex        = 0x0000
     *   wLength       = 0x0008
     *
     * MIPS is big-endian: writing a 32-bit word puts its high byte at the
     * lowest address. The on-wire byte stream for the SETUP packet is
     * 80 06 00 01 00 00 08 00 (wValue and wLength are little-endian on
     * the wire), so we pack accordingly. */
    uint32_t setup_uncached = dma_uncached_base() + BUF_SETUP_OFF;
    usb_hal_io_write(setup_uncached + 0, 0x80060001u);  /* bytes: 80 06 00 01 */
    usb_hal_io_write(setup_uncached + 4, 0x00000800u);  /* bytes: 00 00 08 00 */

    /* Verify what's actually in the buffer (no swap, no parsing — pure
     * byte read-back). If these don't match 80 06 00 01 00 00 08 00,
     * our writes are getting mangled and the device gets garbage. */
    {
        uint32_t w0 = usb_hal_io_read(setup_uncached + 0);
        uint32_t w1 = usb_hal_io_read(setup_uncached + 4);
        uint8_t  b[8] = {
            (uint8_t)(w0 >> 24), (uint8_t)(w0 >> 16),
            (uint8_t)(w0 >>  8), (uint8_t) w0,
            (uint8_t)(w1 >> 24), (uint8_t)(w1 >> 16),
            (uint8_t)(w1 >>  8), (uint8_t) w1,
        };
        usb_hal_log("vusbh11: SETUP buf readback: %02x %02x %02x %02x %02x %02x %02x %02x\n",
               b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
        usb_hal_log("vusbh11:   (expected:        80 06 00 01 00 00 08 00)\n");
    }

    /* Clear RX buffer so we can see what came back. */
    uint32_t rx_uncached = dma_uncached_base() + BUF_RX_OFF;
    usb_hal_io_write(rx_uncached + 0, 0x00000000u);
    usb_hal_io_write(rx_uncached + 4, 0x00000000u);

    /* Configure EP0 control: handshake + RX + TX enabled + HOST_WO_HUB.
     *
     * HOST_WO_HUB tells the controller "no hub between us and the device"
     * so it skips PRE/SPLIT transaction prefixes that are only valid when
     * a hub is acting as repeater for a downstream low-speed device.
     * Without this bit, a directly-attached full-speed device sees the
     * PRE prefix and stays silent. Per Thar0's send_token it's always set
     * (`NO_HUB | bType`).
     *
     * No RETRY_DIS — we want the controller to NAK-retry while the device
     * settles after reset. */
    vusb11_write(s_port, VUSB11_OFF_EP_CTL_BASE + 0u * 4u,
                 VUSB11_EP_HOST_WO_HUB |
                 VUSB11_EP_HSHK_EN | VUSB11_EP_RX_EN | VUSB11_EP_TX_EN);

    /* Device is still on default address 0; FSEN = full-speed signaling. */
    vusb11_write(s_port, VUSB11_OFF_ADDR, VUSB11_ADDR_FSEN | 0u);

    /* Wait for any in-flight token to finish. Should be a no-op right
     * now — we haven't issued anything yet. */
    {
        int spins = 0;
        while (vusb11_read(s_port, VUSB11_OFF_CTL) & VUSB11_CTL_TX_SUSPEND_BUSY) {
            if (++spins > 100000) {
                usb_hal_log("vusbh11: TX_SUSPEND_BUSY stuck, aborting SETUP\n");
                usb_hal_wait_ms(2500);
                return;
            }
        }
    }

    /* Synchronous SETUP → IN → STATUS OUT, all via the single-pong
     * helpers (do_setup_phase / do_in_data_phase / do_status_out_zlp).
     *
     * The earlier inline form re-primed both RX pongs each IN retry and
     * never advanced s_*_next_pong, which desynced the tracker from the
     * controller's actual pong and broke STATUS OUT. Using the same
     * helpers as every later transfer keeps the tracker in lockstep.
     *
     * USB Chapter 9: host must issue the data-phase IN promptly after
     * the SETUP ACK — many devices abandon the request if more than
     * ~10 ms elapses. The helpers run back-to-back, well within that. */

    usb_hal_log("vusbh11: SETUP+IN+STATUS via helpers (GET_DESCRIPTOR DEVICE 8)\n");
    if (!do_setup_phase(0)) {
        usb_hal_log("vusbh11: first SETUP failed\n");
        vusbh11_enum_state = VUSBH11_ENUM_ERROR;
        usb_hal_wait_ms(2500);
        return;
    }

    uint16_t got8 = do_in_data_phase(8);
    if (got8 == 0) {
        usb_hal_log("vusbh11: first IN failed\n");
        vusbh11_enum_state = VUSBH11_ENUM_ERROR;
        usb_hal_wait_ms(3000);
        return;
    }

    {
        uint8_t tmp8[8] = {0};
        read_rx_bytes(tmp8, (got8 < 8) ? got8 : 8);
        for (int i = 0; i < 8; i++) vusbh11_dev_desc[i] = tmp8[i];
        vusbh11_dev_desc_valid = true;
        usb_hal_log("vusbh11: GOT DESC (%u bytes): %02x %02x %02x %02x %02x %02x %02x %02x\n",
               got8,
               vusbh11_dev_desc[0], vusbh11_dev_desc[1],
               vusbh11_dev_desc[2], vusbh11_dev_desc[3],
               vusbh11_dev_desc[4], vusbh11_dev_desc[5],
               vusbh11_dev_desc[6], vusbh11_dev_desc[7]);
        usb_hal_log("vusbh11:   bLength=%u bDescType=%u bcdUSB=%04x\n",
               vusbh11_dev_desc[0], vusbh11_dev_desc[1],
               (unsigned)(vusbh11_dev_desc[2] |
                          (vusbh11_dev_desc[3] << 8)));
        usb_hal_log("vusbh11:   class=%u sub=%u proto=%u maxPkt0=%u\n",
               vusbh11_dev_desc[4], vusbh11_dev_desc[5],
               vusbh11_dev_desc[6], vusbh11_dev_desc[7]);
        vusbh11_enum_state = VUSBH11_ENUM_DONE;

        /* ---- STATUS phase of GET_DESCRIPTOR(DEVICE,8): OUT ZLP ---- */
        usb_hal_log("vusbh11: STATUS OUT ZLP (close first control transfer)\n");
        if (!do_status_out_zlp()) {
            usb_hal_log("vusbh11: STATUS phase failed, aborting further enum\n");
            usb_hal_wait_ms(3000);
            return;
        }
        usb_hal_log("vusbh11: STATUS OUT ACK'd\n");

        /* ---- SET_ADDRESS(1) ---- */
        usb_hal_wait_ms(5);   /* small gap between back-to-back control transfers */
        usb_hal_log("vusbh11: SET_ADDRESS(1)\n");
        {
            uint8_t set_addr_setup[8] = {
                0x00,       /* bmRequestType: OUT | Standard | Device */
                0x05,       /* bRequest: SET_ADDRESS                  */
                0x01, 0x00, /* wValue = 1 (LE)                        */
                0x00, 0x00, /* wIndex = 0                             */
                0x00, 0x00, /* wLength = 0                            */
            };
            write_setup_packet(set_addr_setup);
            vusb11_write(s_port, VUSB11_OFF_ADDR, VUSB11_ADDR_FSEN | 0u);
            if (!do_setup_phase(0)) {
                usb_hal_log("vusbh11: SET_ADDRESS SETUP failed\n");
                usb_hal_wait_ms(3000); return;
            }
            /* SET_ADDRESS has no data phase. STATUS is IN ZLP. */
            if (!do_status_in_zlp()) {
                usb_hal_log("vusbh11: SET_ADDRESS STATUS failed\n");
                usb_hal_wait_ms(3000); return;
            }
            /* Per USB spec, host updates its address register AFTER the
             * status phase completes. The device updates its own address
             * after sending the status ZLP. SetAddress recovery is
             * TSETADDR = 2 ms — wait so device is settled at new addr. */
            vusb11_write(s_port, VUSB11_OFF_ADDR, VUSB11_ADDR_FSEN | 1u);
            s_current_addr = 1;   /* future issue_token calls re-write this */
            usb_hal_wait_ms(5);
            usb_hal_log("vusbh11: SET_ADDRESS(1) done; ADDR=1\n");
        }

        /* ---- GET_DESCRIPTOR(DEVICE, 18) at the new address ---- */
        usb_hal_log("vusbh11: GET_DESCRIPTOR(DEVICE,18) at addr=1\n");
        {
            uint8_t get_full_setup[8] = {
                0x80,       /* bmRequestType: IN | Standard | Device  */
                0x06,       /* bRequest: GET_DESCRIPTOR               */
                0x00, 0x01, /* wValue = 0x0100 (DEVICE descriptor)    */
                0x00, 0x00, /* wIndex = 0                             */
                0x12, 0x00, /* wLength = 18                           */
            };
            write_setup_packet(get_full_setup);
            vusb11_write(s_port, VUSB11_OFF_ADDR, VUSB11_ADDR_FSEN | 1u);
            if (!do_setup_phase(1)) {
                usb_hal_log("vusbh11: GET_DESC(18) SETUP failed at addr=1\n");
                usb_hal_wait_ms(3000); return;
            }
            uint16_t got = do_in_data_phase(18);
            if (got == 0) {
                usb_hal_log("vusbh11: GET_DESC(18) IN failed\n");
                usb_hal_wait_ms(3000); return;
            }
            if (!do_status_out_zlp()) {
                usb_hal_log("vusbh11: GET_DESC(18) STATUS failed\n");
                usb_hal_wait_ms(3000); return;
            }
            uint8_t desc[18] = {0};
            read_rx_bytes(desc, (got < 18) ? got : 18);
            uint16_t vid = (uint16_t)(desc[8]  | (desc[9]  << 8));
            uint16_t pid = (uint16_t)(desc[10] | (desc[11] << 8));
            uint16_t rel = (uint16_t)(desc[12] | (desc[13] << 8));
            usb_hal_log("vusbh11: FULL DESC (%u bytes):\n", got);
            usb_hal_log("  [0..5]   %02x %02x %02x %02x %02x %02x\n",
                   desc[0], desc[1], desc[2], desc[3], desc[4], desc[5]);
            usb_hal_log("  [6..11]  %02x %02x %02x %02x %02x %02x\n",
                   desc[6], desc[7], desc[8], desc[9], desc[10], desc[11]);
            usb_hal_log("  [12..17] %02x %02x %02x %02x %02x %02x\n",
                   desc[12], desc[13], desc[14], desc[15], desc[16], desc[17]);
            usb_hal_log("vusbh11: VID=%04x PID=%04x bcdDevice=%04x\n",
                   vid, pid, rel);
            usb_hal_log("vusbh11: iMfr=%u iProduct=%u iSerial=%u nConfigs=%u\n",
                   desc[14], desc[15], desc[16], desc[17]);
        }

        /* ---- GET_DESCRIPTOR(CONFIG, 9) — get wTotalLength ---- */
        usb_hal_wait_ms(5);
        usb_hal_log("vusbh11: GET_DESCRIPTOR(CONFIG,9)\n");
        uint16_t total_len = 0;
        {
            uint8_t cfg_short[8] = {
                0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x09, 0x00,
            };
            write_setup_packet(cfg_short);
            vusb11_write(s_port, VUSB11_OFF_ADDR, VUSB11_ADDR_FSEN | 1u);
            if (!do_setup_phase(1)) { usb_hal_wait_ms(3000); return; }
            uint16_t got = do_in_data_phase(9);
            if (got == 0) { usb_hal_log("vusbh11: CONFIG short IN failed\n"); usb_hal_wait_ms(3000); return; }
            if (!do_status_out_zlp()) { usb_hal_log("vusbh11: CONFIG short STATUS failed\n"); usb_hal_wait_ms(3000); return; }
            uint8_t c[9] = {0};
            read_rx_bytes(c, (got < 9) ? got : 9);
            total_len = (uint16_t)(c[2] | (c[3] << 8));
            usb_hal_log("vusbh11: CONFIG short OK; wTotalLength=%u nInterfaces=%u\n",
                   total_len, c[4]);
            usb_hal_log("  bConfigValue=%u iConfig=%u bmAttr=%02x bMaxPower=%u (=%u mA)\n",
                   c[5], c[6], c[7], c[8], (unsigned)c[8] * 2);
        }

        /* ---- GET_DESCRIPTOR(CONFIG, total_len) — pull all sub-descriptors ---- */
        usb_hal_wait_ms(5);
        if (total_len > 0 && total_len <= 64) {
            usb_hal_log("vusbh11: GET_DESCRIPTOR(CONFIG,%u) full\n", total_len);
            uint8_t cfg_full[8] = {
                0x80, 0x06, 0x00, 0x02, 0x00, 0x00,
                (uint8_t)(total_len & 0xFFu),
                (uint8_t)((total_len >> 8) & 0xFFu),
            };
            write_setup_packet(cfg_full);
            if (!do_setup_phase(1)) { usb_hal_wait_ms(3000); return; }
            uint16_t got = do_in_data_phase(total_len);
            if (got == 0) { usb_hal_log("vusbh11: CONFIG full IN failed\n"); usb_hal_wait_ms(3000); return; }
            if (!do_status_out_zlp()) { usb_hal_log("vusbh11: CONFIG full STATUS failed\n"); usb_hal_wait_ms(3000); return; }

            /* Parse: walk the descriptor list. Each starts with [bLength,
             * bDescType, ...]. Types we care about:
             *   0x02 CONFIG, 0x04 INTERFACE, 0x05 ENDPOINT */
            uint16_t use = (got < 64) ? got : 64;
            uint8_t buf[64] = {0};
            read_rx_bytes(buf, use);
            usb_hal_log("vusbh11: CONFIG full %u bytes — parsing:\n", use);

            uint16_t off = 0;
            while (off + 2 <= use) {
                uint8_t blen = buf[off + 0];
                uint8_t btyp = buf[off + 1];
                if (blen == 0 || off + blen > use) break;
                if (btyp == 0x04 && blen >= 9) {
                    /* INTERFACE: bInterfaceNumber, ..., bInterfaceClass,
                     *  bInterfaceSubClass, bInterfaceProtocol */
                    usb_hal_log("  IF #%u: class=%02x sub=%02x proto=%02x nEPs=%u\n",
                           buf[off + 2], buf[off + 5], buf[off + 6],
                           buf[off + 7], buf[off + 4]);
                } else if (btyp == 0x05 && blen >= 7) {
                    /* ENDPOINT: bEndpointAddress, bmAttributes, wMaxPacketSize, bInterval */
                    uint8_t addr = buf[off + 2];
                    uint8_t attr = buf[off + 3];
                    uint16_t mps = (uint16_t)(buf[off + 4] | (buf[off + 5] << 8));
                    const char *dir = (addr & 0x80) ? "IN " : "OUT";
                    const char *xfer = "?";
                    switch (attr & 0x03) {
                    case 0: xfer = "CTRL"; break;
                    case 1: xfer = "ISO";  break;
                    case 2: xfer = "BULK"; break;
                    case 3: xfer = "INT";  break;
                    }
                    usb_hal_log("  EP%02x %s %s mps=%u\n",
                           addr & 0x0F, dir, xfer, mps);
                }
                off += blen;
            }
        } else {
            usb_hal_log("vusbh11: total_len=%u out of range, skipping full read\n",
                   total_len);
        }

        /* ---- SET_CONFIGURATION(1) ---- */
        usb_hal_wait_ms(5);
        usb_hal_log("vusbh11: SET_CONFIGURATION(1)\n");
        {
            uint8_t set_cfg[8] = {
                0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
            };
            write_setup_packet(set_cfg);
            if (!do_setup_phase(1)) { usb_hal_wait_ms(3000); return; }
            if (!do_status_in_zlp()) { usb_hal_wait_ms(3000); return; }
            /* With single-pong control transfers, pong tracking stays
             * in sync naturally; no ODD_RST needed. Bulk EP toggles are
             * already 0 (= DATA0, what the device resets to on SET_CONFIG)
             * via static init. */
            usb_hal_log("vusbh11: SET_CONFIGURATION(1) done — endpoints active\n");
        }

        /* ---- SCSI INQUIRY via Bulk-Only Transport ---- */
        usb_hal_wait_ms(10);
        usb_hal_log("vusbh11: --- Mass Storage Class: starting INQUIRY ---\n");
        if (!send_inquiry()) {
            usb_hal_log("vusbh11: INQUIRY failed — aborting block-read tests\n");
            usb_hal_wait_ms(3000); return;
        }
        usb_hal_log("vusbh11: INQUIRY succeeded\n");

        /* ---- SCSI TEST_UNIT_READY ---- */
        /* Many Mass Storage devices require a successful TUR before
         * any media-touching command (READ_CAPACITY counts). Some
         * also use the first TUR to clear an "attention" condition. */
        usb_hal_wait_ms(10);
        usb_hal_log("vusbh11: --- SCSI TEST_UNIT_READY ---\n");
        {
            uint8_t tur_cdb[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            uint16_t got = 0;
            uint8_t st = send_bot_in_cmd(tur_cdb, 6, BUF_BULK_IN_OFF, 0, &got);
            usb_hal_log("vusbh11: TUR status=%u (0=ready)\n", st);
        }

        /* Pre-flight REQUEST_SENSE to clear any "needs to clear sense"
         * state some drives latch after attach. We don't care about the
         * response; just consume it. */
        usb_hal_wait_ms(10);
        {
            uint8_t cdb[6] = { 0x03, 0x00, 0x00, 0x00, 0x12, 0x00 };
            uint16_t got = 0;
            (void)send_bot_in_cmd(cdb, 6, BUF_BULK_IN_OFF, 18, &got);
        }

        /* ---- SCSI READ_CAPACITY_10 ---- */
        /* OVER-REQUEST workaround: ask for 64 bytes via CBW even though
         * actual READ_CAPACITY response is 8 bytes. Some buggy BOT devices
         * skip the data phase when Hi=Di exactly; they want Hi>Di so they
         * can terminate with a short packet + residue. Linux usb-storage
         * does this. If real data shows up, this is the device-quirk fix. */
        usb_hal_wait_ms(10);
        usb_hal_log("vusbh11: --- SCSI READ_CAPACITY_10 (over-request 64) ---\n");
        uint32_t block_count = 0, block_size = 0;
        {
            uint8_t cdb[10] = {
                0x25, /* READ_CAPACITY_10 */
                0x00,
                0x00, 0x00, 0x00, 0x00, /* LBA = 0 */
                0x00, 0x00,
                0x00, /* PMI = 0 */
                0x00, /* control */
            };
            /* Pre-zero data buffer so we can tell genuine writes from stale. */
            {
                uint32_t b = dma_uncached_base() + BUF_BULK_IN_OFF;
                for (int i = 0; i < 64; i += 4) usb_hal_io_write(b + i, 0u);
            }
            uint16_t got = 0;
            uint8_t status = send_bot_in_cmd(cdb, 10, BUF_BULK_IN_OFF, 64, &got);
            uint8_t cap[16] = {0};
            read_buf_bytes(cap, BUF_BULK_IN_OFF, 16);
            usb_hal_log("vusbh11: READ_CAPACITY status=%u got=%u\n", status, got);
            usb_hal_log("  data [0..15]: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n",
                   cap[0], cap[1], cap[2], cap[3], cap[4], cap[5], cap[6], cap[7],
                   cap[8], cap[9], cap[10], cap[11], cap[12], cap[13], cap[14], cap[15]);
            if (status == 0 && got >= 8) {
                /* Response is 4 bytes LBA-of-last-block (BIG-endian)
                 * + 4 bytes block-length (BIG-endian). */
                uint32_t last_lba = ((uint32_t)cap[0] << 24) |
                                    ((uint32_t)cap[1] << 16) |
                                    ((uint32_t)cap[2] << 8)  |
                                    ((uint32_t)cap[3]);
                block_size = ((uint32_t)cap[4] << 24) |
                             ((uint32_t)cap[5] << 16) |
                             ((uint32_t)cap[6] << 8)  |
                             ((uint32_t)cap[7]);
                block_count = last_lba + 1;
                uint32_t mb = (uint32_t)(((uint64_t)block_count * block_size) >> 20);
                usb_hal_log("vusbh11: capacity: %lu blocks x %lu B = ~%lu MB\n",
                       (unsigned long)block_count,
                       (unsigned long)block_size,
                       (unsigned long)mb);
            }
        }

        /* Publish geometry — vusbh11_disk_read/write gate on these
         * being nonzero. Only commit if READ_CAPACITY succeeded and the
         * device reports a sector size we can handle (we support 512;
         * non-512 drives are exceedingly rare and would need chunking). */
        if (block_size == 512 && block_count > 0) {
            vusbh11_block_size  = block_size;
            vusbh11_block_count = block_count;
        }
    }
}

/* Wait until TX_SUSPEND_BUSY clears so we can safely write TOKEN. */
static bool wait_token_busy_clear(void)
{
    int spins = 0;
    while (vusb11_read(s_port, VUSB11_OFF_CTL) & VUSB11_CTL_TX_SUSPEND_BUSY) {
        if (++spins > 1000000) return false;
    }
    return true;
}

/* Re-issue per-token state: EP control flags then the token. ADDR is NOT
 * touched here — caller is responsible for setting ADDR before calling.
 *
 * IMPORTANT: in host mode the controller only consults ENDPT_HOST_RG[0]
 * (the EP0 control register) regardless of which device endpoint the
 * TOKEN is addressing. The other ENDPT[n] registers are effectively
 * unused. Always (re)configure ENDPT[0] here.
 *
 * RETRY_DIS is set so NAKs come back to us as a TOKDNE with PID=NAK
 * rather than the controller silently retrying forever. We want visibility
 * into negotiation failures, and we'll schedule retries from software. */
static void issue_token(uint8_t token_byte)
{
    if (!wait_token_busy_clear()) {
        usb_hal_log("vusbh11: TX_SUSPEND_BUSY stuck before TOKEN=%02x\n", token_byte);
        return;
    }
    vusb11_write(s_port, VUSB11_OFF_EP_CTL_BASE + 0u,
                 VUSB11_EP_HOST_WO_HUB | VUSB11_EP_RETRY_DIS |
                 VUSB11_EP_HSHK_EN | VUSB11_EP_RX_EN | VUSB11_EP_TX_EN);
    /* Always re-write ADDRESS per Thar0's send_token pattern. The bit 7
     * (LSEN) stays 0 for full-speed; bits 6:0 carry the device address. */
    vusb11_write(s_port, VUSB11_OFF_ADDR, VUSB11_ADDR_FSEN | s_current_addr);

    /* Clear the per-direction completion flag for the direction we're
     * about to use. Otherwise wait_for_completion would see leftover
     * pending from a prior transaction and return immediately with stale
     * w0. Token PID high nibble: 0x9 = IN (host RX), 0x1 = OUT, 0xD =
     * SETUP (both are host TX). */
    uint8_t pid = (uint8_t)((token_byte >> 4) & 0xFu);
    if (pid == 0x9u) s_rx_tokdne_pending = 0;
    else             s_tx_tokdne_pending = 0;

    vusb11_write(s_port, VUSB11_OFF_TOKEN, token_byte);
}

/* ----- Generic byte-buffer reader/writer for the MMIO BDT region ----- */

static void write_bytes_to_buf(uint32_t buf_off, const uint8_t *src, uint16_t n)
{
    uint32_t base = dma_uncached_base() + buf_off;
    for (uint16_t i = 0; i < n; i += 4) {
        uint32_t w = 0;
        for (uint16_t j = 0; j < 4 && (i + j) < n; j++) {
            w |= ((uint32_t)src[i + j]) << ((3 - j) * 8);
        }
        usb_hal_io_write(base + i, w);
    }
}

static void read_buf_bytes(uint8_t *dst, uint32_t buf_off, uint16_t n)
{
    /* Align to 4 bytes per io_read; buf_off may itself be unaligned
     * (e.g. BUF_BULK_IN_OFF + 510 for an MBR sig check). */
    uint32_t base = dma_uncached_base();
    for (uint16_t i = 0; i < n; i++) {
        uint32_t addr    = base + buf_off + i;
        uint32_t aligned = addr & ~3u;
        uint32_t w       = usb_hal_io_read(aligned);
        dst[i] = (uint8_t)(w >> ((3 - (addr & 3u)) * 8));
    }
}

/* ----- Bulk transfer helpers ----- */

/* Per-endpoint data toggle (USB spec tracks DATA0/DATA1 per endpoint per
 * direction). Reset to 0 (DATA0) when the configuration is set, then
 * toggled after each successful transaction. Used for the DATA01 bit in
 * the BDT CTRL byte, and with DTS=1 the controller enforces a match
 * between this and the device's wire PID. */
static uint8_t s_ep_out_toggle[16] = {0};
static uint8_t s_ep_in_toggle[16]  = {0};

/* Pong selector tracking. In host mode the controller uses a SINGLE pair
 * of BDT slots (EP0 Even/Odd for each direction) for all transactions,
 * and alternates between them after every successful packet. We must
 * track this so we know which slot to prime for the next transaction —
 * priming the wrong one means the controller writes to a stale buffer
 * pointer (or silently drops the transfer). Reset to 0 (Even) by an
 * ODD_RST pulse at the SET_CONFIGURATION boundary. */
static uint8_t s_rx_next_pong = 0;   /* 0=Even, 1=Odd */
static uint8_t s_tx_next_pong = 0;


/* Bulk OUT: send `n` bytes from a buffer at `buf_off` to endpoint `ep`.
 * Single-pong + DTS=1 + per-EP toggle tracking, per Thar0's vusbh11ma.c
 * (host mode requires this discipline; dual-pong + DTS=0 works for the
 * first control transfer but breaks after a few back-to-back bulk
 * transactions because the controller's internal pong selector drifts
 * out of sync with our software assumptions).
 *
 * IMPORTANT: in host mode the controller's BDT lookup ALWAYS uses EP0
 * slots regardless of the TOKEN's endpoint field. The `ep` parameter
 * only affects the on-wire endpoint number in the TOKEN packet. */
static bool bulk_out(uint8_t ep, uint32_t buf_off, uint16_t n)
{
    uint32_t buf_phys = dma_phys_base() + buf_off;
    uint32_t off = s_tx_next_pong ? BDT_EP0_TX_ODD_OFF : BDT_EP0_TX_EVEN_OFF;
    uint8_t  ctrl = s_ep_out_toggle[ep & 0xFu] ? 0xC8u : 0x88u;
    prime_bdt_slot(off, ctrl, n, buf_phys);
    issue_token(VUSB11_TOKEN_OUT(ep));

    uint32_t w0 = wait_for_completion(false);   /* OUT is host TX */
    if (w0 == 0u) {
        usb_hal_log("vusbh11: bulk OUT EP%u timed out\n", ep);
        return false;
    }
    uint8_t pid = (uint8_t)(((w0 >> 24) >> 2) & 0xFu);
    if (pid != 0x2u) {
        usb_hal_log("vusbh11: bulk OUT EP%u not ACK'd, pid=%x\n", ep, pid);
        return false;
    }
    s_ep_out_toggle[ep & 0xFu] ^= 1u;
    return true;
}

/* Bulk IN: receive up to `max_n` bytes into the buffer at `buf_off` from
 * endpoint `ep`. NAK-retry loop up to 50 attempts. Returns received byte
 * count on success, 0 on failure/STALL/timeout. */
static uint16_t bulk_in(uint8_t ep, uint32_t buf_off, uint16_t max_n)
{
    uint32_t buf_phys = dma_phys_base() + buf_off;
    uint8_t  toggle   = s_ep_in_toggle[ep & 0xFu];
    uint8_t  ctrl     = toggle ? 0xC8u : 0x88u;

    for (int attempt = 0; attempt < 50; attempt++) {
        uint32_t off = s_rx_next_pong ? BDT_EP0_RX_ODD_OFF : BDT_EP0_RX_EVEN_OFF;
        prime_bdt_slot(off, ctrl, max_n, buf_phys);
        issue_token(VUSB11_TOKEN_IN(ep));

        uint32_t w0 = wait_for_completion(true);
        if (w0 == 0u) {
            usb_hal_log("vusbh11: bulk IN EP%u timed out (att %d, toggle %u)\n",
                   ep, attempt, toggle);
            return 0;
        }
        uint8_t pid = (uint8_t)(((w0 >> 24) >> 2) & 0xFu);
        if (pid == 0xAu) { usb_hal_wait_ms(10); continue; }
        if (pid == 0xEu) { usb_hal_log("vusbh11: bulk IN EP%u STALL\n", ep); return 0; }
        if (pid != 0x3u && pid != 0xBu) {
            usb_hal_log("vusbh11: bulk IN EP%u unexpected pid=%x\n", ep, pid);
            return 0;
        }
        s_ep_in_toggle[ep & 0xFu] ^= 1u;
        return (uint16_t)(((w0 >> 8) & 0xFFu) | ((w0 & 0x03u) << 8));
    }
    usb_hal_log("vusbh11: bulk IN EP%u gave up after 50 NAKs\n", ep);
    return 0;
}

/* Receive `want_n` bytes IN from `ep` into MMIO buffer at `buf_off`, by
 * looping single-packet bulk_in calls. Stops early on short packet
 * (device signaling end of transfer) or error. Returns total bytes
 * received (could be < want_n on short response). */
static uint16_t bulk_in_multi(uint8_t ep, uint32_t buf_off, uint16_t want_n,
                              uint16_t mps)
{
    uint16_t total = 0;
    while (total < want_n) {
        /* Thar0's vusbh11ma.c primes RX with BC=0xFF, not BC=mps. Match
         * his pattern. The buffer in MMIO has plenty of room. */
        uint16_t got = bulk_in(ep, buf_off + total, 0xFFu);
        if (got == 0) break;       /* error / NAK exhaustion / STALL */
        total += got;
        if (got < mps) break;      /* short packet → end of data */
    }
    return total;
}

/* Send `want_n` bytes OUT to `ep` from the buffer at `buf_off`, by looping
 * single-packet bulk_out calls of up to `mps` bytes each. Returns total
 * bytes successfully transmitted; if a bulk_out fails partway through, we
 * stop and return whatever was sent up to that point.
 *
 * USB Mass Storage devices expect the last packet of the data phase to
 * be exactly `expected` bytes or a short packet — the BOT layer in the
 * caller is responsible for sizing the transfer to match the CBW's
 * dCBWDataTransferLength. */
static uint16_t bulk_out_multi(uint8_t ep, uint32_t buf_off, uint16_t want_n,
                               uint16_t mps)
{
    uint16_t total = 0;
    while (total < want_n) {
        uint16_t chunk = (uint16_t)(want_n - total);
        if (chunk > mps) chunk = mps;
        if (!bulk_out(ep, buf_off + total, chunk)) break;
        total += chunk;
    }
    return total;
}

/* Issue an arbitrary SCSI command via BOT. `cdb` is the 6/10/12/16-byte
 * Command Descriptor Block. `data_in_buf_off` is where to receive data
 * (or 0 if there's no data phase). `expected` is the expected data length.
 * Returns the CSW status byte (0=PASSED, 1=FAILED, 2=PHASE_ERROR) or
 * 0xFF if the BOT transaction itself failed.
 *
 * Only data-IN commands supported for now (the ones we need for read). */
static uint8_t send_bot_in_cmd(const uint8_t *cdb, uint8_t cdb_len,
                                uint32_t data_buf_off, uint32_t expected,
                                uint16_t *received_out)
{
    static uint32_t tag = 1;
    tag++;
    /* Build CBW (31 bytes). */
    uint8_t cbw[31] = {0};
    /* dCBWSignature = 0x43425355 LE → wire bytes "USBC" = 55 53 42 43 */
    cbw[0] = 0x55; cbw[1] = 0x53; cbw[2] = 0x42; cbw[3] = 0x43;
    /* dCBWTag LE */
    cbw[4] = (uint8_t)(tag);       cbw[5] = (uint8_t)(tag >> 8);
    cbw[6] = (uint8_t)(tag >> 16); cbw[7] = (uint8_t)(tag >> 24);
    /* dCBWDataTransferLength LE */
    cbw[8]  = (uint8_t)(expected);
    cbw[9]  = (uint8_t)(expected >> 8);
    cbw[10] = (uint8_t)(expected >> 16);
    cbw[11] = (uint8_t)(expected >> 24);
    /* bmCBWFlags = 0x80 (data IN) */
    cbw[12] = 0x80;
    /* bCBWLUN, bCBWCBLength */
    cbw[13] = 0;
    cbw[14] = cdb_len;
    /* CBWCB (16 bytes, padded) */
    for (uint8_t i = 0; i < cdb_len && i < 16; i++) cbw[15 + i] = cdb[i];

    write_bytes_to_buf(BUF_CBW_OFF, cbw, 31);

    /* Read CBW back from MMIO and verify it matches what we wrote.
     * If they differ, write_bytes_to_buf has a subtle bug. */
    {
        uint8_t back[31] = {0};
        read_buf_bytes(back, BUF_CBW_OFF, 31);
        bool ok = true;
        for (int i = 0; i < 31; i++) {
            if (back[i] != cbw[i]) { ok = false; break; }
        }
        usb_hal_log("  CBW(%s, cdb[0]=%02x): %02x %02x %02x %02x | %02x %02x %02x %02x | %02x %02x %02x %02x | %02x %02x %02x | %02x %02x %02x %02x %02x %02x\n",
               ok ? "OK" : "MISMATCH", cdb[0],
               back[0], back[1], back[2], back[3],
               back[4], back[5], back[6], back[7],
               back[8], back[9], back[10], back[11],
               back[12], back[13], back[14],
               back[15], back[16], back[17], back[18], back[19], back[20]);
    }

    if (!bulk_out(2, BUF_CBW_OFF, 31)) {
        usb_hal_log("vusbh11: CBW OUT failed (cdb[0]=0x%02x)\n", cdb[0]);
        return 0xFFu;
    }

    /* Data IN phase. */
    uint16_t got = 0;
    if (expected > 0) {
        got = bulk_in_multi(1, data_buf_off, (uint16_t)expected, 64);
        if (got == 0 && expected > 0) {
            usb_hal_log("vusbh11: data IN failed for cdb[0]=0x%02x\n", cdb[0]);
            /* Still try to read CSW so we get a clean state. */
        }
    }
    if (received_out) *received_out = got;

    /* CSW phase — always one 13-byte packet on EP1 IN. */
    {
        uint32_t b = dma_uncached_base() + BUF_CSW_OFF;
        for (int i = 0; i < 16; i += 4) usb_hal_io_write(b + i, 0u);
    }
    uint16_t csw_got = bulk_in(1, BUF_CSW_OFF, 64);
    if (csw_got == 0) {
        usb_hal_log("vusbh11: CSW IN failed (cdb[0]=0x%02x)\n", cdb[0]);
        return 0xFFu;
    }
    uint8_t csw[13] = {0};
    read_buf_bytes(csw, BUF_CSW_OFF, (csw_got < 13) ? csw_got : 13);
    usb_hal_log("  CSW(got %u): %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
           csw_got,
           csw[0], csw[1], csw[2], csw[3], csw[4], csw[5], csw[6],
           csw[7], csw[8], csw[9], csw[10], csw[11], csw[12]);
    bool sig_ok = (csw[0] == 0x55 && csw[1] == 0x53 &&
                   csw[2] == 0x42 && csw[3] == 0x53);
    uint32_t csw_tag = (uint32_t)csw[4]  | ((uint32_t)csw[5] << 8) |
                       ((uint32_t)csw[6] << 16) | ((uint32_t)csw[7] << 24);
    uint8_t  status = csw[12];
    if (!sig_ok) {
        usb_hal_log("vusbh11: CSW signature bad: %02x %02x %02x %02x\n",
               csw[0], csw[1], csw[2], csw[3]);
        return 0xFFu;
    }
    if (csw_tag != tag) {
        usb_hal_log("vusbh11: CSW tag mismatch: got %lu, expected %lu\n",
               (unsigned long)csw_tag, (unsigned long)tag);
    }
    return status;
}

/* OUT-direction BOT command: mirror of send_bot_in_cmd but with bulk OUT
 * data phase (host → device). `data_buf_off` is where the data to send
 * lives; `length` is the byte count (must equal dCBWDataTransferLength).
 * Returns the CSW status byte, or 0xFF on BOT failure.
 *
 * Caller is responsible for filling data_buf_off with the payload
 * (write_bytes_to_buf) BEFORE calling. */
static uint8_t send_bot_out_cmd(const uint8_t *cdb, uint8_t cdb_len,
                                 uint32_t data_buf_off, uint32_t length,
                                 uint16_t *sent_out)
{
    static uint32_t tag = 0x10000u;   /* keep distinct from IN cmd tag space */
    tag++;
    uint8_t cbw[31] = {0};
    cbw[0] = 0x55; cbw[1] = 0x53; cbw[2] = 0x42; cbw[3] = 0x43;  /* "USBC" */
    cbw[4] = (uint8_t)(tag);       cbw[5] = (uint8_t)(tag >> 8);
    cbw[6] = (uint8_t)(tag >> 16); cbw[7] = (uint8_t)(tag >> 24);
    cbw[8]  = (uint8_t)(length);
    cbw[9]  = (uint8_t)(length >> 8);
    cbw[10] = (uint8_t)(length >> 16);
    cbw[11] = (uint8_t)(length >> 24);
    cbw[12] = 0x00;  /* bmCBWFlags = 0x00 (data OUT) */
    cbw[13] = 0;     /* LUN */
    cbw[14] = cdb_len;
    for (uint8_t i = 0; i < cdb_len && i < 16; i++) cbw[15 + i] = cdb[i];

    write_bytes_to_buf(BUF_CBW_OFF, cbw, 31);

    {
        uint8_t back[31] = {0};
        read_buf_bytes(back, BUF_CBW_OFF, 31);
        usb_hal_log("  CBW-OUT(cdb[0]=%02x len=%lu): %02x %02x %02x %02x | %02x %02x %02x %02x | %02x %02x %02x %02x | %02x %02x %02x | %02x %02x %02x %02x %02x %02x\n",
               cdb[0], (unsigned long)length,
               back[0], back[1], back[2], back[3],
               back[4], back[5], back[6], back[7],
               back[8], back[9], back[10], back[11],
               back[12], back[13], back[14],
               back[15], back[16], back[17], back[18], back[19], back[20]);
    }

    if (!bulk_out(2, BUF_CBW_OFF, 31)) {
        usb_hal_log("vusbh11: CBW OUT failed (cdb[0]=0x%02x)\n", cdb[0]);
        return 0xFFu;
    }

    /* Data OUT phase. */
    uint16_t sent = 0;
    if (length > 0) {
        sent = bulk_out_multi(2, data_buf_off, (uint16_t)length, 64);
        if (sent != length) {
            usb_hal_log("vusbh11: data OUT short for cdb[0]=0x%02x (sent %u of %lu)\n",
                   cdb[0], sent, (unsigned long)length);
            /* Continue to CSW anyway so we can see what the device thinks. */
        }
    }
    if (sent_out) *sent_out = sent;

    /* CSW phase. */
    {
        uint32_t b = dma_uncached_base() + BUF_CSW_OFF;
        for (int i = 0; i < 16; i += 4) usb_hal_io_write(b + i, 0u);
    }
    uint16_t csw_got = bulk_in(1, BUF_CSW_OFF, 64);
    if (csw_got == 0) {
        usb_hal_log("vusbh11: CSW IN failed (cdb[0]=0x%02x)\n", cdb[0]);
        return 0xFFu;
    }
    uint8_t csw[13] = {0};
    read_buf_bytes(csw, BUF_CSW_OFF, (csw_got < 13) ? csw_got : 13);
    usb_hal_log("  CSW(got %u): %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
           csw_got,
           csw[0], csw[1], csw[2], csw[3], csw[4], csw[5], csw[6],
           csw[7], csw[8], csw[9], csw[10], csw[11], csw[12]);
    bool sig_ok = (csw[0] == 0x55 && csw[1] == 0x53 &&
                   csw[2] == 0x42 && csw[3] == 0x53);
    uint32_t csw_tag = (uint32_t)csw[4]  | ((uint32_t)csw[5] << 8) |
                       ((uint32_t)csw[6] << 16) | ((uint32_t)csw[7] << 24);
    uint8_t  status = csw[12];
    if (!sig_ok) {
        usb_hal_log("vusbh11: CSW signature bad: %02x %02x %02x %02x\n",
               csw[0], csw[1], csw[2], csw[3]);
        return 0xFFu;
    }
    if (csw_tag != tag) {
        usb_hal_log("vusbh11: CSW tag mismatch: got %lu, expected %lu\n",
               (unsigned long)csw_tag, (unsigned long)tag);
    }
    return status;
}

/* Build + send a SCSI INQUIRY through Bulk-Only Transport. Returns true
 * on full success (CBW ACKed + INQUIRY data received + CSW status=passed). */
static bool send_inquiry(void)
{
    /* Build CBW (31 bytes, wire byte order). */
    uint8_t cbw[31] = {
        /* dCBWSignature 0x43425355 LE → wire bytes "USBC" = 55 53 42 43 */
        0x55, 0x53, 0x42, 0x43,
        /* dCBWTag = 1 (LE) */
        0x01, 0x00, 0x00, 0x00,
        /* dCBWDataTransferLength = 36 (LE) */
        0x24, 0x00, 0x00, 0x00,
        /* bmCBWFlags = 0x80 (data IN) */
        0x80,
        /* bCBWLUN = 0 */
        0x00,
        /* bCBWCBLength = 6 (SCSI-6 INQUIRY) */
        0x06,
        /* CBWCB (16 bytes; first 6 are the SCSI INQUIRY command) */
        0x12, 0x00, 0x00, 0x00, 0x24, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };
    write_bytes_to_buf(BUF_CBW_OFF, cbw, 31);

    usb_hal_log("vusbh11: BOT: sending CBW (INQUIRY)\n");
    if (!bulk_out(2, BUF_CBW_OFF, 31)) {
        usb_hal_log("vusbh11: CBW transmit failed\n");
        return false;
    }

    usb_hal_log("vusbh11: BOT: reading INQUIRY data\n");
    uint16_t got_data = bulk_in(1, BUF_BULK_IN_OFF, 64);
    if (got_data == 0) {
        usb_hal_log("vusbh11: INQUIRY data IN failed\n");
        return false;
    }
    usb_hal_log("vusbh11: got %u bytes of INQUIRY response\n", got_data);

    /* Parse INQUIRY response. */
    uint8_t inq[36] = {0};
    read_buf_bytes(inq, BUF_BULK_IN_OFF, (got_data < 36) ? got_data : 36);

    char vendor[9]  = {0};
    char product[17] = {0};
    char rev[5]      = {0};
    for (int i = 0; i < 8;  i++) vendor[i]  = (char)inq[8 + i];
    for (int i = 0; i < 16; i++) product[i] = (char)inq[16 + i];
    for (int i = 0; i < 4;  i++) rev[i]     = (char)inq[32 + i];

    usb_hal_log("vusbh11: SCSI dev_type=%u removable=%u version=0x%02x\n",
           inq[0] & 0x1F, (inq[1] >> 7) & 1, inq[2]);
    usb_hal_log("vusbh11:   vendor='%s'\n", vendor);
    usb_hal_log("vusbh11:   product='%s'\n", product);
    usb_hal_log("vusbh11:   rev='%s'\n", rev);

    /* Read CSW (13 bytes). Zero the buffer first so stale data is
     * visible if the controller doesn't actually write to it. */
    {
        uint32_t b = dma_uncached_base() + BUF_CSW_OFF;
        for (int i = 0; i < 16; i += 4) usb_hal_io_write(b + i, 0u);
    }
    usb_hal_log("vusbh11: BOT: reading CSW\n");
    uint16_t got_csw = bulk_in(1, BUF_CSW_OFF, 64);
    if (got_csw == 0) {
        usb_hal_log("vusbh11: CSW read failed\n");
        return false;
    }
    uint8_t csw[13] = {0};
    read_buf_bytes(csw, BUF_CSW_OFF, (got_csw < 13) ? got_csw : 13);
    usb_hal_log("vusbh11: CSW (got %u): %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
           got_csw,
           csw[0], csw[1], csw[2], csw[3], csw[4], csw[5], csw[6],
           csw[7], csw[8], csw[9], csw[10], csw[11], csw[12]);
    /* dCSWSignature should be 0x53425355 LE → wire bytes 55 53 42 53 ("USBS") */
    bool sig_ok = (csw[0] == 0x55 && csw[1] == 0x53 &&
                   csw[2] == 0x42 && csw[3] == 0x53);
    uint32_t tag    = (uint32_t)csw[4]  | ((uint32_t)csw[5] << 8)  |
                      ((uint32_t)csw[6] << 16) | ((uint32_t)csw[7] << 24);
    uint32_t resid  = (uint32_t)csw[8]  | ((uint32_t)csw[9] << 8)  |
                      ((uint32_t)csw[10] << 16) | ((uint32_t)csw[11] << 24);
    uint8_t  status = csw[12];
    usb_hal_log("vusbh11: CSW sig_ok=%d tag=%lu resid=%lu status=%u (%s)\n",
           sig_ok ? 1 : 0, (unsigned long)tag, (unsigned long)resid, status,
           (status == 0) ? "PASSED" :
           (status == 1) ? "FAILED" :
           (status == 2) ? "PHASE ERROR" : "?");
    return sig_ok && (status == 0);
}

/* (wait_bdt_done removed — transactions now complete via the ISR. See
 * wait_for_completion() above for the ISR-driven equivalent.) */


void vusbh11_poll(void)
{
    if (!s_initted) return;

    if (vusbh11_attach_pending) {
        vusbh11_attach_pending = false;
        process_attach();
        /* process_attach itself drives SETUP+IN synchronously and sets
         * vusbh11_enum_state to DONE or ERROR. */
    }

    /* Refresh BDT word0 readbacks for the status screen. */
    vusbh11_bdt_ep0_txeven_w0 = usb_hal_io_read(bdt_uncached_base() + BDT_EP0_TX_EVEN_OFF);
    vusbh11_bdt_ep0_rxodd_w0  = usb_hal_io_read(bdt_uncached_base() + BDT_EP0_RX_ODD_OFF);
}

void vusbh11_snapshot(vusbh11_snapshot_t *out)
{
    out->istat   = vusb11_read(s_port, VUSB11_OFF_ISTAT);
    out->inten   = vusb11_read(s_port, VUSB11_OFF_INTEN);
    out->errstat = vusb11_read(s_port, VUSB11_OFF_ERRSTAT);
    out->ctl     = vusb11_read(s_port, VUSB11_OFF_CTL);
    out->addr    = vusb11_read(s_port, VUSB11_OFF_ADDR);
    out->otgstat = vusb11_read(s_port, VUSB11_OFF_OTGSTAT);
    out->otgctl  = vusb11_read(s_port, VUSB11_OFF_OTGCTL);
    out->reg10   = vusb11_read(s_port, VUSB11_OFF_REG10);
    out->stat    = vusb11_read(s_port, VUSB11_OFF_STAT);

    uint32_t l = vusb11_read(s_port, VUSB11_OFF_FRMNUML) & 0xFFu;
    uint32_t h = vusb11_read(s_port, VUSB11_OFF_FRMNUMH) & 0x07u;
    out->frmnum = (h << 8) | l;
}

void vusbh11_isr(void)
{
    vusbh11_isr_count++;

    uint32_t r10     = vusb11_read(s_port, VUSB11_OFF_REG10);
    uint32_t istat   = vusb11_read(s_port, VUSB11_OFF_ISTAT);
    uint32_t errstat = vusb11_read(s_port, VUSB11_OFF_ERRSTAT);
    uint32_t otgstat = vusb11_read(s_port, VUSB11_OFF_OTGSTAT);

    vusbh11_last_reg10   = r10;
    vusbh11_last_istat   = istat;
    vusbh11_last_errstat = errstat;
    vusbh11_last_otgstat = otgstat;

    /* Count events. We do NOT do bottom-half work here — see HANDOFF.md
     * for why (ISR must stay short to not miss SOFs).
     *
     * Note we do NOT increment attach_count on the level-asserted ATTACH
     * bit here — that happens inside the "ATTACHEN was armed" block below
     * so we only count GENUINE attach events. */
    if (istat & VUSB11_ISTAT_RST)        vusbh11_rst_count++;
    if (istat & VUSB11_ISTAT_TOKEN_DONE) {
        vusbh11_tokdne_count++;
        /* STAT register holds EP/DIR/PONG of the token that just
         * completed. Capture it now — STAT gets overwritten by the next
         * transaction. ERRSTAT we already read above. */
        uint32_t stat = vusb11_read(s_port, VUSB11_OFF_STAT);
        vusbh11_last_tokdne_stat    = stat;
        vusbh11_last_tokdne_errstat = errstat;

        /* Bottom-half of transaction handling: read the BDT slot the
         * controller just consumed, save its w0, advance SW pong, and
         * signal the waiting app via the per-direction pending flag.
         *
         * Direction comes from STAT bit 3: in host mode the controller
         * empirically sets bit 3=1 when the last transaction was host-TX
         * (OUT/SETUP), 0 when host-RX (IN). We trust SW pong for the
         * slot (we don't read STAT.ODD); each TOKDNE advances pong by 1
         * matching the controller's internal Even/Odd flip.
         *
         * Doing this in the ISR rather than polling avoids the
         * TXSUSPENDTOKENBUSY race that broke back-to-back bulk IN
         * packets — by the time TOKDNE fires, the controller is fully
         * idle and its pong selector has been advanced. */
        bool was_host_tx = (stat & 0x08u) != 0u;
        uint32_t bdt = bdt_uncached_base();
        if (was_host_tx) {
            uint32_t off = s_tx_next_pong ? BDT_EP0_TX_ODD_OFF : BDT_EP0_TX_EVEN_OFF;
            s_tx_last_w0 = usb_hal_io_read(bdt + off);
            s_tx_next_pong ^= 1u;
            s_tx_tokdne_pending = 1;
        } else {
            uint32_t off = s_rx_next_pong ? BDT_EP0_RX_ODD_OFF : BDT_EP0_RX_EVEN_OFF;
            s_rx_last_w0 = usb_hal_io_read(bdt + off);
            s_rx_next_pong ^= 1u;
            s_rx_tokdne_pending = 1;
        }
    }
    if (istat & VUSB11_ISTAT_SOFTOK)     vusbh11_softok_count++;
    if (istat & VUSB11_ISTAT_ERROR)      vusbh11_error_count++;

    /* If ATTACH is asserting, mask ATTACHEN immediately. The bit reflects
     * "device is still attached", not an edge, so it re-fires as soon as
     * we W1C it. Masking the enable bit stops the IRQ storm and lets a
     * polled bottom-half (main loop) drive bus reset / speed detect /
     * enumeration at its own pace. */
    /* The ATTACH bit is level-asserted while the device stays plugged in
     * — every subsequent ISR (TOKDNE, ERROR, USBRST) re-sees it. Only act
     * on a GENUINE attach event: ATTACH bit set AND ATTACHEN currently
     * armed. After the first attach we mask ATTACHEN so this branch will
     * stay false until the next plug/unplug cycle re-arms it. */
    {
        uint32_t inten_now = vusb11_read(s_port, VUSB11_OFF_INTEN);
        if ((istat & VUSB11_ISTAT_ATTACH) &&
            (inten_now & VUSB11_INTEN_ATTACHEN)) {
            vusb11_write(s_port, VUSB11_OFF_INTEN,
                         inten_now & ~VUSB11_INTEN_ATTACHEN);
            vusbh11_attach_count++;
            vusbh11_attach_pending = true;
        }
    }

    /* W1C clear. */
    if (errstat) vusb11_write(s_port, VUSB11_OFF_ERRSTAT, errstat);
    if (istat)   vusb11_write(s_port, VUSB11_OFF_ISTAT,   istat);
    if (r10)     vusb11_write(s_port, VUSB11_OFF_REG10,   r10);
}

/* ===== Public disk I/O API =====
 *
 * These are the entry points iquesync_iodev wraps into gz's iodev. They
 * assume process_attach() has already run successfully (vusbh11_block_size
 * non-zero). Each call issues ONE SCSI READ_10 / WRITE_10 for up to N
 * sectors, then copies in/out of the caller's buffer via uncached MMIO.
 *
 * Buffer size limit: our bulk DMA buffer is 8 KB minus 0x100 of header =
 * 7936 bytes = 15 sectors. We chunk to 8 sectors per command for headroom
 * (matches what process_attach's test READ exercised, and stays well
 * within the 64-byte * 64-packet bulk packet count). */

#define VUSBH11_MAX_CHUNK_BLOCKS  8u

/* Pull n bytes out of the bulk MMIO buffer into a caller buffer. */
static void copy_from_bulk(uint8_t *dst, uint32_t buf_off, uint32_t n)
{
    uint32_t b = dma_uncached_base() + buf_off;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t w = usb_hal_io_read(b + (i & ~3u));
        dst[i] = (uint8_t)(w >> (24 - 8 * (i & 3)));
    }
}

/* Push n bytes from a caller buffer into the bulk MMIO buffer. */
static void copy_to_bulk(uint32_t buf_off, const uint8_t *src, uint32_t n)
{
    uint32_t b = dma_uncached_base() + buf_off;
    /* Read-modify-write 32-bit words so we never disturb adjacent bytes. */
    uint32_t i = 0;
    while (i < n) {
        uint32_t word_off = i & ~3u;
        uint32_t w = usb_hal_io_read(b + word_off);
        for (int byte_in_word = (int)(i & 3u);
             byte_in_word < 4 && i < n;
             byte_in_word++, i++)
        {
            uint32_t shift = 24 - 8 * byte_in_word;
            w = (w & ~(0xFFu << shift)) | ((uint32_t)src[i] << shift);
        }
        usb_hal_io_write(b + word_off, w);
    }
}

static int do_read_chunk(uint32_t lba, uint16_t n_blocks, uint8_t *dst)
{
    uint8_t cdb[10] = {
        0x28, /* READ_10 */
        0x00,
        (uint8_t)((lba >> 24) & 0xFFu),
        (uint8_t)((lba >> 16) & 0xFFu),
        (uint8_t)((lba >>  8) & 0xFFu),
        (uint8_t)( lba        & 0xFFu),
        0x00,
        (uint8_t)((n_blocks >> 8) & 0xFFu),
        (uint8_t)( n_blocks       & 0xFFu),
        0x00,
    };
    uint32_t want = (uint32_t)n_blocks * 512u;
    uint16_t got = 0;
    uint8_t status = send_bot_in_cmd(cdb, 10, BUF_BULK_IN_OFF, want, &got);
    if (status != 0 || got != want)
        return -1;
    copy_from_bulk(dst, BUF_BULK_IN_OFF, want);
    return 0;
}

static int do_write_chunk(uint32_t lba, uint16_t n_blocks, const uint8_t *src)
{
    uint32_t want = (uint32_t)n_blocks * 512u;
    copy_to_bulk(BUF_BULK_OUT_OFF, src, want);
    uint8_t cdb[10] = {
        0x2A, /* WRITE_10 */
        0x00,
        (uint8_t)((lba >> 24) & 0xFFu),
        (uint8_t)((lba >> 16) & 0xFFu),
        (uint8_t)((lba >>  8) & 0xFFu),
        (uint8_t)( lba        & 0xFFu),
        0x00,
        (uint8_t)((n_blocks >> 8) & 0xFFu),
        (uint8_t)( n_blocks       & 0xFFu),
        0x00,
    };
    uint16_t sent = 0;
    uint8_t status = send_bot_out_cmd(cdb, 10, BUF_BULK_OUT_OFF, want, &sent);
    if (status != 0 || sent != want)
        return -1;
    return 0;
}

int vusbh11_disk_read(uint32_t lba, uint32_t n_blocks, void *dst)
{
    if (vusbh11_block_size != 512 || vusbh11_block_count == 0)
        return -1;
    if (lba + n_blocks > vusbh11_block_count)
        return -1;
    uint8_t *p = (uint8_t *)dst;
    while (n_blocks > 0) {
        uint16_t chunk = (n_blocks > VUSBH11_MAX_CHUNK_BLOCKS)
                       ? (uint16_t)VUSBH11_MAX_CHUNK_BLOCKS
                       : (uint16_t)n_blocks;
        if (do_read_chunk(lba, chunk, p) != 0)
            return -1;
        lba      += chunk;
        n_blocks -= chunk;
        p        += (uint32_t)chunk * 512u;
    }
    return 0;
}

int vusbh11_disk_write(uint32_t lba, uint32_t n_blocks, const void *src)
{
    if (vusbh11_block_size != 512 || vusbh11_block_count == 0)
        return -1;
    if (lba + n_blocks > vusbh11_block_count)
        return -1;
    const uint8_t *p = (const uint8_t *)src;
    while (n_blocks > 0) {
        uint16_t chunk = (n_blocks > VUSBH11_MAX_CHUNK_BLOCKS)
                       ? (uint16_t)VUSBH11_MAX_CHUNK_BLOCKS
                       : (uint16_t)n_blocks;
        if (do_write_chunk(lba, chunk, p) != 0)
            return -1;
        lba      += chunk;
        n_blocks -= chunk;
        p        += (uint32_t)chunk * 512u;
    }
    return 0;
}

#endif /* Z64_VERSION == Z64_OOTIQC */
