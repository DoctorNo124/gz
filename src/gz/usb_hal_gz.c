/**
 * @file usb_hal_gz.c
 * @brief gz/iQue backend for usb_hal.h.
 *
 * gz patches into OoT-CN at 0x80400000 and links against OoT's libultra
 * runtime via lib/liboot-ique-cn.a (a linker script exposing OoT symbols
 * at their fixed addresses). gz's source uses its own libultra-compatible
 * header set (n64/...) supplied by glankk's n64 toolchain, NOT
 * thar0-ultralib's PR/... header set the upstream iquesync-host was
 * written against.
 *
 * This backend is the gz-flavoured equivalent of iquesync-host's
 * usb_hal_libultra.c:
 *
 *  - Interrupts: one libultra kernel thread per USB port, blocked on a
 *    message queue fed by osSetEventMesg(OS_EVENT_USBn, ...). On each
 *    USB event the thread invokes the registered C ISR.
 *
 *  - OS_EVENT_USB0/1 are NOT in glankk's n64/message.h (which only
 *    declares the standard N64 events 0..14), but iQue libultra in
 *    OoT-CN extends __osEventStateTab and the MIPS exception vector to
 *    handle events 27/28 — see thar0-ultralib src/os/exceptasm.s:675-697.
 *    Defined locally; trust the runtime.
 *
 *  - MI mask: poke MI_BB_MASK (0xA430003C) directly. Same bits libdragon
 *    exposes via mi.h.
 *
 *  - Timing: read CP0 COUNT directly; iQue CPU=72 MHz so COUNT=36 MHz.
 *
 *  - is_ique: __osBbIsBb is declared in gz's z64.h and resolved at link
 *    time against OoT's data segment.
 *
 *  - Logging: stub. Can be wired to gz's printf (rdb-based) for debug
 *    builds later.
 */

#include "z64.h"
#if Z64_VERSION == Z64_OOTIQC

#ifndef USB_HAL_BACKEND_GZ
#  define USB_HAL_BACKEND_GZ
#endif
#include <stdarg.h>
#include <stdint.h>
#include <n64.h>
#include "usb_hal.h"
#include "util.h"   /* maybe_init_gp() — required at thread entry */

/* iQue libultra adds these event slots beyond glankk's <n64/message.h>
 * range (0..14). Values from thar0-ultralib/include/PR/os_message.h. */
#define USB_HAL_OS_EVENT_USB0   27
#define USB_HAL_OS_EVENT_USB1   28

/* iQue CPU = 72 MHz; CP0 COUNT increments at half rate = 36 MHz. */
#define USB_HAL_BB_COUNT_HZ     36000000u

/* Direct register addresses (same constants libdragon's mi.h uses). */
#define USB_HAL_MI_BB_MASK      ((volatile uint32_t *)0xA430003C)
#define USB_HAL_MI_SET_USB0     0x00200000u
#define USB_HAL_MI_CLR_USB0     0x00100000u
#define USB_HAL_MI_SET_USB1     0x00800000u
#define USB_HAL_MI_CLR_USB1     0x00400000u

bool usb_hal_is_ique(void)
{
    return __osBbIsBb != 0;
}

static inline uint32_t read_count(void)
{
    uint32_t c;
    __asm__ volatile ("mfc0 %0, $9" : "=r"(c));
    return c;
}

void usb_hal_wait_ms(uint32_t ms)
{
    uint32_t cycles_per_ms = USB_HAL_BB_COUNT_HZ / 1000u;
    uint32_t start = read_count();
    uint32_t target = ms * cycles_per_ms;
    while ((read_count() - start) < target) {
        /* busy wait */
    }
}

/* ---- ISR bridge thread ---- */

#define USB_HAL_ISR_STACK_SIZE   0x800
#define USB_HAL_ISR_MQ_DEPTH     8
#define USB_HAL_ISR_PRIORITY     OS_PRIORITY_RMON  /* 250 — above app threads */

static OSThread          s_isr_thread[2];
static __attribute__((aligned(8))) uint8_t s_isr_stack[2][USB_HAL_ISR_STACK_SIZE];
static OSMesgQueue       s_isr_mq[2];
static OSMesg            s_isr_mesg[2][USB_HAL_ISR_MQ_DEPTH];
static usb_hal_isr_fn_t  s_isr_fn[2];

static void isr_thread_entry(void *arg)
{
    /* gz places statics in .sdata/.sbss and accesses them gp-relative.
     * Each thread must initialize its $gp before touching any global
     * (otherwise the first gp-relative load faults — observed live as a
     * TLB-load exception on a sign-extended-from-zero offset). Same
     * pattern rdb_main uses at rdb.c:664. */
    maybe_init_gp();

    int port = (int)(uintptr_t)arg;
    for (;;) {
        OSMesg m;
        osRecvMesg(&s_isr_mq[port], &m, OS_MESG_BLOCK);
        if (s_isr_fn[port] != NULL)
            s_isr_fn[port]();
        /* iQue libultra's MI/RCP dispatcher MASKS the USB MI bit after
         * posting the event (write of MI_USBn_CLR to MI_BB_MASK; see
         * thar0-ultralib src/os/exceptasm.s:675-697). The driver only
         * gets ONE interrupt unless we re-arm it. Without this re-enable
         * we saw exactly attach_count=1, tokdne_count=0 — ATTACH fired
         * once, every subsequent TOKDNE was silently swallowed. */
        usb_hal_irq_enable(port, true);
    }
}

void usb_hal_irq_install(int port, usb_hal_isr_fn_t isr)
{
    int p = port & 1;
    s_isr_fn[p] = isr;
    osCreateMesgQueue(&s_isr_mq[p], s_isr_mesg[p], USB_HAL_ISR_MQ_DEPTH);
    OSEvent ev = (p == 0) ? USB_HAL_OS_EVENT_USB0 : USB_HAL_OS_EVENT_USB1;
    osSetEventMesg(ev, &s_isr_mq[p], (OSMesg)(uintptr_t)p);
    osCreateThread(&s_isr_thread[p], 100 + p,
                   isr_thread_entry, (void *)(uintptr_t)p,
                   &s_isr_stack[p][USB_HAL_ISR_STACK_SIZE],
                   USB_HAL_ISR_PRIORITY);
    osStartThread(&s_isr_thread[p]);
}

void usb_hal_irq_remove(int port)
{
    /* gz never tears down USB at runtime. API symmetry only. */
    s_isr_fn[port & 1] = NULL;
}

void usb_hal_irq_enable(int port, bool on)
{
    if ((port & 1) == 0) {
        *USB_HAL_MI_BB_MASK = on ? USB_HAL_MI_SET_USB0 : USB_HAL_MI_CLR_USB0;
    } else {
        *USB_HAL_MI_BB_MASK = on ? USB_HAL_MI_SET_USB1 : USB_HAL_MI_CLR_USB1;
    }
}

void usb_hal_irq_rearm(int port)
{
    /* Re-register OS event → message queue binding. libultra's
     * __osEventStateTab is a flat array indexed by event number;
     * any caller that does osSetEventMesg with the same event ID
     * overwrites our binding silently. Re-calling here ensures
     * USB IRQs deliver to OUR queue regardless of what else
     * happened in between. Doesn't touch the thread or queue,
     * so no state corruption. */
    int p = port & 1;
    OSEvent ev = (p == 0) ? USB_HAL_OS_EVENT_USB0 : USB_HAL_OS_EVENT_USB1;
    osSetEventMesg(ev, &s_isr_mq[p], (OSMesg)(uintptr_t)p);
    /* Re-arm MI as well, in case it was masked. */
    usb_hal_irq_enable(port, true);
}

void usb_hal_log(const char *fmt, ...)
{
    /* Route to libultra's osSyncPrintf, which on iQue SK firmware
     * pushes bytes over the consumer-cable USB0 device interface.
     * aulon2 on the PC sees them as console output, so every
     * usb_hal_log(...) line in vusbh11.c shows up live. Same pattern
     * iquesync-host uses (src/usb_hal_libultra.c:133).
     *
     * osSyncPrintf is variadic with no vprintf-style entry, so format
     * into a local buffer first and pass that as a literal "%s". 512
     * bytes covers all our log lines (longest is the BDT dump). */
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    /* libultra ships its own vsprintf in libgultra (xstdio). */
    extern int vsprintf(char *str, const char *format, va_list ap);
    int n = vsprintf(buf, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n >= (int)sizeof(buf)) buf[sizeof(buf) - 1] = '\0';
    /* iQue libultra signature. glankk's n64 headers don't declare it,
     * but it's present in liboot-ique-cn.a — declare here to satisfy
     * the compiler. */
    extern void osSyncPrintf(const char *fmt, ...);
    osSyncPrintf("%s", buf);
}

#endif /* Z64_VERSION == Z64_OOTIQC */
