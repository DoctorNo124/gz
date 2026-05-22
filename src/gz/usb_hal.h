/**
 * @file usb_hal.h
 * @brief Hardware abstraction layer for the iQue USB host stack.
 *
 * The vusb11 / vusbh11 driver sources don't include <libdragon.h> (nor
 * <ultra64.h>) directly — they only depend on this small HAL. To port
 * the USB stack to a new environment, write a usb_hal_<env>.c that
 * provides the four extern functions below; the inline trivials need
 * no porting.
 *
 *   build target          | links against
 *   ----------------------+-----------------------
 *   iquesync-host (here)  | usb_hal_libdragon.c
 *   gz / libultra targets | usb_hal_libultra.c (TBD)
 */
#ifndef IQUESYNC_USB_HAL_H
#define IQUESYNC_USB_HAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Endian swap a 32-bit word — used for BDT buffer-pointer commit.
 * Safe to inline: pure C, identical on every MIPS target. */
static inline uint32_t usb_hal_swab32(uint32_t x)
{
    return __builtin_bswap32(x);
}

/* ---- MMIO read/write ----
 *
 * Empirically: wrapping libdragon's io_read/io_write in a same-prototype
 * wrapper function (even one GCC tail-calls) breaks USB enumeration on
 * iQue — SETUP tokens fail with BUSTIMEOUT. We don't fully understand
 * why a `j io_read` indirection differs from a direct call, but the
 * symptom is reproducible and the fix is to bypass the wrapper.
 *
 * So usb_hal_io_read/io_write are exposed as MACROS that expand to the
 * environment's primitives directly. The "HAL" still exists at the
 * source level — vusb11.c / vusbh11.c only reference `usb_hal_io_read`
 * — but at compile time it's a verbatim libdragon (or libultra) call.
 * No function-call layer between us and the environment's accessor. */
#if defined(USB_HAL_BACKEND_GZ)
   /* gz on iQue: gz ships its own libultra-compatible toolchain
    * (mips64-ultra-elf) with n64/... headers, not thar0-ultralib's
    * PR/os.h. The equivalent of __osDisableInt is util.h's set_irqf. */
#  include "util.h"
   static inline uint32_t usb_hal_io_read(uint32_t addr) {
       int sr = set_irqf(0);
       uint32_t v = *(volatile uint32_t *)(uintptr_t)addr;
       set_irqf(sr);
       return v;
   }
   static inline void usb_hal_io_write(uint32_t addr, uint32_t val) {
       int sr = set_irqf(0);
       *(volatile uint32_t *)(uintptr_t)addr = val;
       set_irqf(sr);
   }
#elif defined(USB_HAL_BACKEND_LIBULTRA)
#  include <PR/os.h>
#  include <PR/os_internal_reg.h>
   static inline uint32_t usb_hal_io_read(uint32_t addr) {
       OSIntMask sr = __osDisableInt();
       uint32_t v = *(volatile uint32_t *)(uintptr_t)addr;
       __osRestoreInt(sr);
       return v;
   }
   static inline void usb_hal_io_write(uint32_t addr, uint32_t val) {
       OSIntMask sr = __osDisableInt();
       *(volatile uint32_t *)(uintptr_t)addr = val;
       __osRestoreInt(sr);
   }
#elif defined(USB_HAL_BACKEND_LIBDRAGON_NATIVE)
   /* TEST backend 1 — same access pattern as the libultra backend (direct
    * KSEG1 32-bit access guarded by interrupt-disable), but uses
    * libdragon's disable_interrupts/enable_interrupts. Selectable in
    * the libdragon ROM so we can validate that the libultra-style MMIO
    * approach actually works on iQue hardware BEFORE we ship it into
    * gz. If iquesync-host built with -DUSB_HAL_BACKEND_LIBDRAGON_NATIVE
    * passes full enum + WRITE_10 round-trip, the libultra backend is
    * highly likely to work too. */
#  include <libdragon.h>
   static inline uint32_t usb_hal_io_read(uint32_t addr) {
       disable_interrupts();
       uint32_t v = *(volatile uint32_t *)(uintptr_t)addr;
       enable_interrupts();
       return v;
   }
   static inline void usb_hal_io_write(uint32_t addr, uint32_t val) {
       disable_interrupts();
       *(volatile uint32_t *)(uintptr_t)addr = val;
       enable_interrupts();
   }
#elif defined(USB_HAL_BACKEND_LIBDRAGON_SR_TEST)
   /* TEST backend 2 — same access pattern as the libultra backend, but
    * uses inline assembly that does EXACTLY what libultra's pre-J
    * __osDisableInt / __osRestoreInt do (mfc0/mtc0 on the Status
    * register's IE bit). This is the most rigorous validation of the
    * libultra MMIO pattern we can do from a libdragon ROM: every
    * instruction in the critical section matches what the libultra
    * backend will execute.
    *
    * libultra's pre-J __osDisableInt:
    *   mfc0 t0, $12         ; read Status
    *   and  t1, t0, ~SR_IE  ; clear IE bit (bit 0)
    *   mtc0 t1, $12         ; write Status
    *   andi v0, t0, SR_IE   ; return saved IE
    *
    * libultra's __osRestoreInt:
    *   mfc0 t0, $12         ; read Status
    *   or   t0, t0, a0      ; OR in saved IE
    *   mtc0 t0, $12         ; write Status
    *
    * (The J-version __osDisableInt does the same SR.IE flip plus some
    * scheduler-state bookkeeping involving __OSGlobalIntMask and
    * __osRunningThread. That bookkeeping doesn't affect MMIO
    * correctness — it just notifies libultra's scheduler that
    * interrupts have changed. We're not running under libultra here,
    * so we don't need it; the SR.IE flip alone is the load-bearing
    * part for keeping MMIO atomic with respect to interrupts.) */
#  include <libdragon.h>
   static inline uint32_t usb_hal_sr_disable_int(void) {
       uint32_t sr;
       __asm__ volatile ("mfc0 %0, $12\n\t nop" : "=r"(sr));
       uint32_t new_sr = sr & ~1u;
       __asm__ volatile ("mtc0 %0, $12\n\t nop\n\t nop" :: "r"(new_sr));
       return sr & 1u;
   }
   static inline void usb_hal_sr_restore_int(uint32_t saved) {
       uint32_t sr;
       __asm__ volatile ("mfc0 %0, $12\n\t nop" : "=r"(sr));
       sr |= saved;
       __asm__ volatile ("mtc0 %0, $12\n\t nop\n\t nop" :: "r"(sr));
   }
   static inline uint32_t usb_hal_io_read(uint32_t addr) {
       uint32_t sr = usb_hal_sr_disable_int();
       uint32_t v = *(volatile uint32_t *)(uintptr_t)addr;
       usb_hal_sr_restore_int(sr);
       return v;
   }
   static inline void usb_hal_io_write(uint32_t addr, uint32_t val) {
       uint32_t sr = usb_hal_sr_disable_int();
       *(volatile uint32_t *)(uintptr_t)addr = val;
       usb_hal_sr_restore_int(sr);
   }
#else
   /* Default: libdragon io_read/io_write (production for iquesync-host). */
#  include <libdragon.h>
#  define usb_hal_io_read(addr)        io_read(addr)
#  define usb_hal_io_write(addr, val)  io_write((addr), (val))
#endif

/* ---- Extern functions (environment-provided) ---- */

/* True iff running on iQue (BB) hardware. The USB stack only works on
 * iQue — vusb11_hw_enable() uses this to fail fast on plain N64. */
bool usb_hal_is_ique(void);

/* Block for approximately `ms` milliseconds. */
void usb_hal_wait_ms(uint32_t ms);

/* Install / remove an interrupt handler for one of the two BB USB
 * controllers. `port` is 0 (USB0) or 1 (USB1). Handler runs in
 * interrupt context. */
typedef void (*usb_hal_isr_fn_t)(void);
void usb_hal_irq_install(int port, usb_hal_isr_fn_t isr);
void usb_hal_irq_remove(int port);

/* Unmask / mask the IRQ line at the MI level. Installing a handler is
 * not sufficient — the MI_BB_MASK bit for the controller must also be
 * set before interrupts will reach the CPU. Call install + enable(true)
 * at startup, and (for cleanup) enable(false) + remove. */
void usb_hal_irq_enable(int port, bool on);

/* printf-style diagnostic log. Environments without a log channel may
 * stub this to no-op. */
void usb_hal_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

#endif /* IQUESYNC_USB_HAL_H */
