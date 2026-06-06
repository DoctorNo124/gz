# iQue USB Host Driver — Technical Specification

A clean-room specification for implementing a USB host stack on the iQue
Player capable of mounting a USB Mass Storage (MSC) flash drive as a
FAT-formatted block device.

This document captures the architectural shape of the driver and the
non-obvious engineering decisions that arose during development. It is
written so that an implementer can produce a working driver from this
spec, the USB 2.0 specification, the USB MSC + Bulk-Only Transport
(BOT) specifications, and the iQue's published hardware register map —
without reference to any other USB driver source.

---

## 1. Scope

The driver lets the iQue act as a USB host, enumerate a single
full-speed USB Mass Storage device on one of the two on-board USB
controllers, and expose 512-byte block-aligned read/write to it. No
hub support, no isochronous transfers, no low-speed devices.

A FAT filesystem layer sits on top of the block interface; that layer
is out of scope here.

---

## 2. Target Hardware

The iQue's USB block is two instances of a K20-family USB OTG IP block.
Implementers should treat the following as the authoritative starting
points:

- The iQue's interrupt-mask register (`MI_BB_MASK`) and the per-port
  enable bits.
- The two USB controller register windows (USB0 and USB1) — each
  exposes a standard K20-family USB SIE + BDT layout.
- A small per-port "wrapper" region (clock gating, wrapper-level
  status).

A K20 USB module reference manual is the cleanest source for register
semantics. This document does not repeat register definitions; it
describes architectural decisions that aren't captured there.

Two things that aren't in the public manual but are needed:

- The OTG control register's "host mode" value is a specific bitmask
  (D+/D- pulldowns + OTG enable + an additional reserved bit).
  Implementers will need to derive or observe the correct value.
- The MI mask register uses **SET/CLR bit-pair semantics**: writing
  bit *N* sets a mask bit; writing bit *N+1* clears it. Readback
  reports the current state at a third bit position. This is easy to
  get wrong from a casual reading.

---

## 3. Architecture

Four layers, top to bottom:

```
+-----------------------------------------------------------+
| Filesystem (FAT) — out of scope                            |
+-----------------------------------------------------------+
| iodev binding — disk_init / disk_read / disk_write         |
+-----------------------------------------------------------+
| SCSI / BOT (Bulk-Only Transport)                           |
|   CBW + data + CSW; INQUIRY / TUR / READ_CAPACITY /        |
|   READ_10 / WRITE_10; BOMSR (class reset) for recovery.    |
+-----------------------------------------------------------+
| USB host SIE driver (vusbh11)                              |
|   Chapter-9 enumeration, control + bulk transfers,         |
|   BDT management, NAK/STALL handling.                      |
+-----------------------------------------------------------+
| HAL                                                        |
|   MMIO read/write, IRQ install/enable/rearm, sleep, log.   |
+-----------------------------------------------------------+
```

The HAL is the only layer that touches the host OS (libultra,
libdragon, …). Everything above the HAL is portable C.

### 3.1 HAL surface

```c
bool     usb_hal_is_ique(void);
void     usb_hal_wait_ms(uint32_t ms);
uint32_t usb_hal_io_read (uint32_t addr);
void     usb_hal_io_write(uint32_t addr, uint32_t val);

typedef void (*usb_hal_isr_fn_t)(void);
void     usb_hal_irq_install(int port, usb_hal_isr_fn_t isr);
void     usb_hal_irq_remove (int port);
void     usb_hal_irq_enable (int port, bool on);
void     usb_hal_irq_rearm  (int port);          // see §5.3

void     usb_hal_log(const char *fmt, ...);
```

**Important — `usb_hal_io_read`/`io_write` should be macros that
expand directly to the environment's MMIO primitive,** not function
calls. Adding even a single tail-called wrapper around the MMIO has
been observed to break enumeration (SETUP tokens fail to reach the
wire). The cause is not fully understood but the symptom is
reproducible; treat this as a constraint.

---

## 4. Initialization & lifecycle

### 4.1 Two reset-state machines, not one

There are **three independent** "reset" concepts the driver must
distinguish:

| Reset           | Scope                                | Frequency           |
|-----------------|--------------------------------------|---------------------|
| Per-transaction | TX_SUSPEND_BUSY drain, status clears | Between SETUP retries |
| Bus reset       | SE0 hold + USB_EN power cycle        | Per ATTACH (controller-level) |
| Host reboot     | Full OS re-init (threads orphaned)   | User-triggered soft-reset |

Conflating any two of these leads to subtle bugs. Specifically:

- The per-transaction recovery must **not** touch `USB_EN`, because
  toggling `USB_EN` on this platform briefly cycles VBUS — which
  power-cycles the device mid-enum.
- The controller bring-up *does* toggle `USB_EN` (it must, see §10.3),
  and this is the only reliable way in hardware to clear a stuck
  `TX_SUSPEND_BUSY`.
- Host reboot orphans the ISR bridge thread (see §5.4) — neither of
  the lower-level resets handles this.

### 4.2 Split idempotency

The driver's `init(port)` function is called every time the FAT layer
mounts the disk — which can happen many times per session (mount,
unmount-on-error, "reset disk" menu action, …). It must be safe to
call repeatedly. But naively reinstalling the OS-level ISR thread on
every call corrupts the OS scheduler (re-osCreateThread'ing into a
struct whose thread is currently blocked on `osRecvMesg` clobbers
internal queue linkage). The fix is to **split the work**:

- **One-shot work** (gated on a `s_initted` flag): create the ISR
  bridge thread and message queue; register the OS event handler.
- **Per-call work** (runs every time): controller `ctlr_init` +
  `host_mode` + `irq_enable`.

The per-call work re-arms `INTEN.ATTACHEN` (which the ISR masked after
the first attach — see §5.2) and re-writes the OTG control register,
which causes the OTG block to re-assert `ISTAT.ATTACH` for the
still-connected device. The fresh `ATTACH` IRQ fires, the existing
bridge thread receives it, and enumeration runs again.

---

## 5. Interrupt model

### 5.1 Why a bridge thread

USB ISR work cannot run on the OS's IRQ stack: many operations
(scheduling enumeration work, calling into BOT, blocking on a SETUP
retry) require thread context. The standard pattern is:

1. The hardware ISR runs very briefly (read `ISTAT`, set software
   flags, W1C-clear bits, return).
2. The OS event handler posts a message to a per-port queue.
3. A bridge thread blocked on that queue wakes up, runs the C-level
   handler (`vusbh11_isr`), which inspects the flags and queues
   bottom-half work.
4. A main-loop pump (`vusbh11_poll`) sees the queued work and runs the
   heavy lifting (`process_attach`, …).

### 5.2 ATTACH is level-asserted

`ISTAT.ATTACH` reflects "a device is currently plugged in," **not** an
edge. Writing 1 to clear it does not deactivate the bit — it
immediately re-asserts. If `ATTACHEN` is unmasked, this creates an
infinite IRQ storm on a single plug-in.

**Solution:** in the ISR, when `ATTACH & ATTACHEN` is true, immediately
mask `ATTACHEN`. Set a `attach_pending` software flag. The main-loop
bottom-half clears the flag and runs `process_attach`. The user must
re-arm `ATTACHEN` (via §4.2's per-call work) before the next mount
attempt — without that, the next session never gets a fresh ATTACH
event.

### 5.3 Re-arm after foreign interference

Other code in the host environment may overwrite the OS's
event-handler binding (the iQue OS event table is a flat array
indexed by event number; any caller that does `osSetEventMesg` with
the same event ID silently steals the binding). The HAL exposes
`usb_hal_irq_rearm(port)` to re-register the binding without touching
the thread or queue. Call this defensively at the start of every
`disk_init`.

### 5.4 Host reboot orphans the bridge thread

If the host environment supports a full system reboot (e.g. an NMI
reset that re-runs the OS bootstrap), the OS thread scheduler is
re-initialized — but the driver's RAM (including the bridge thread's
`OSThread` struct and the driver's `s_initted` flag) typically
persists. The thread struct is no longer linked into the fresh
scheduler's queues, and `s_initted=true` prevents recreation. USB
IRQs land in a queue that no thread services; ATTACH never fires.

**Solution:** the iodev's reset hook must clear `s_initted` (and
invalidate the FS mount) before triggering the reboot. The plain
cached store reaches RDRAM as long as the reset path performs a full
dcache writeback before the reboot — which the iQue's reset routine
already does. No special cache handling needed.

If the reset routine does not flush the dcache, the implementer must
either:

- Issue an explicit dcache writeback before reboot, or
- Write the flag through the uncached KSEG1 alias of its address.

---

## 6. Controller bring-up

The K20-family register names below are public.

### 6.1 Bring-up order

The order matters. The successfully-derived sequence:

1. **MI gate "init phase"** — mask the per-port MI bit while the
   controller is being reconfigured.
2. **Quiesce** — `CTL = 0` (USB disabled), `ERREN = 0`, `INTEN = 0`.
3. **W1C clear** — `ERRSTAT = 0xFF`, `ISTAT = 0xFF`.
4. **`CTL = ODD_RST`** — resets the controller's BDT EVEN/ODD bank
   tracking.
5. **`ADDR = FSEN | 0`** — default address, full-speed.
6. **BDT page pointers** — split the BDT physical base address into
   `BDTPAGE1/2/3` bytes.
7. **Endpoint 0 control** — clear, then set to `RETRY_DIS | HSHK_EN |
   RX_EN | TX_EN` (see §6.3 for why `RETRY_DIS`).
8. **OTG control** — write the host-mode bitmask (D+/D- pulldowns +
   OTG enable + the reserved bit specific to this PHY).
9. **SOF threshold** — `10 + max_packet_size(FS)`, i.e. `10 + 64 = 74`.
10. **`INTEN = ATTACHEN`** — enable only ATTACH at this stage; tokens
    and errors are masked until `process_attach` runs (see §7).
11. **Re-arm MI** — un-mask the per-port MI bit.

ADDR and EP_CTL set **before** the bus reset is critical. If they're
written afterward the controller may have already moved its state
machine forward and silently ignores the writes.

### 6.2 Bus-reset (port_bringup) sequence

Called from `process_attach` on a fresh ATTACH event:

1. **Settle** — wait `pre_reset_settle_ms` (see §6.4).
2. **Configure** — `ERREN = 0xFF`, BDT pages, `ADDR = FSEN | 0`,
   `EP_CTL[0] = RETRY_DIS | HSHK_EN | RX_EN | TX_EN`.
3. **Drive reset** — `CTL = RESET | HOST_MODE_EN` (USB_EN off).
4. **Hold** — wait `reset_hold_ms` (10ms; matches USB spec minimum
   TRSTRCY).
5. **Two-step release** — write `CTL = HOST_MODE_EN` (release reset,
   USB_EN still off), then `CTL = HOST_MODE_EN | USB_EN`. Doing this
   as one write has been observed to leave stale BDT pong state; keep
   the two-step.
6. **Post-reset settle** — wait `post_reset_ms` (50ms).
7. **Clear sticky status** — `ISTAT/ERRSTAT/wrapper_status = 0xFF`.
8. **`INTEN = TOKDNEEN | ERROREN`** — enable transaction-level IRQs
   (ATTACHEN is now masked because the device is already attached).
9. **Zero the BDT** — 4 EP0 entries.
10. **Prime SETUP buffer** — first GET_DESCRIPTOR(DEVICE, 8) packet.
11. **Reset SW pong trackers** — both to Even.

### 6.3 `RETRY_DIS` on the endpoint

Set `RETRY_DIS` on the EP0 control word for every host-mode token
issue. This matches the public Freescale K20 USB host pattern: the
SIE's bus-timeout machinery and the host's SW retry layer are
designed to be separately responsible, and clearing `RETRY_DIS`
interacts poorly with that division on this IP block.

### 6.4 Cold-boot device-warmup timing

The minimum reset hold is 10ms (USB spec TRSTRCY). The minimum
post-reset settle is also small. But **`pre_reset_settle_ms` must be
much larger for real flash drives** — typically 1–2 seconds.

A USB flash drive's internal MCU often is not fully booted at the
moment the host first sees VBUS rise. SETUP tokens issued during that
window get no response (the device's USB engine isn't running). The
symptom is BUSTIMEOUT on every SETUP retry, looking exactly like a
dead device.

**2000ms `pre_reset_settle_ms`** has been observed to make cold-boot
enumeration reliable for typical consumer USB drives. With this value,
no outer retry loop above `process_attach` is needed for the cold-boot
case.

---

## 7. Enumeration (USB Chapter 9)

Standard Chapter 9 sequence: GET_DESCRIPTOR(DEVICE, 8) → SET_ADDRESS →
GET_DESCRIPTOR(DEVICE, 18) → GET_DESCRIPTOR(CONFIG, 9) →
GET_DESCRIPTOR(CONFIG, full) → SET_CONFIGURATION(1).

### 7.1 SETUP retry budget

The first SETUP after a bus reset is the single most fragile
transaction in the driver. The recommended budget is **10 retries**
with `retry_recovery_soft` (TX_SUSPEND_BUSY drain + sticky-status
clear + ~50ms gap) between attempts. This is per attach event — there
is no benefit to an additional outer attempt loop above
`process_attach`, because that loop would only repeat the same
recovery work `process_attach` already does.

### 7.2 Single-attempt enumeration is correct

The driver's `disk_init` should be **event-driven and single-shot**:
trigger the bring-up, then wait for `process_attach` to publish block
geometry (`block_size == 512 && block_count > 0`). If it doesn't
within ~5 seconds, fail.

This matches the design of any standard USB host stack — repeated full
bus-reset+enum cycles are not how OS USB stacks handle a flaky device.
They retry at the per-transaction level (the SIE's auto-retry +
software NAK retry) and report an error if that fails.

### 7.3 BDT pong + DATA0/DATA1 toggle

The K20 SIE uses BDT double-buffering ("pong" — EVEN/ODD slots) for
each direction of each endpoint. The host must track an SW "next
pong" cursor that mirrors the SIE's hardware cursor, and prime the
correct slot. The hardware cursor resets to Even on bus reset; the SW
cursor must agree.

Separately, the DATA0/DATA1 toggle (PID 0x3 / 0xB on the wire for IN,
0x4D ctrl-word bit for primed-OUT) is per-endpoint per-direction. For
control transfers the SETUP is always DATA0, the data phase starts at
DATA1 and alternates per packet, and the STATUS phase is always
DATA1.

---

## 8. Bulk transfers and NAK strategy

### 8.1 NAK semantics

A NAK on a bulk endpoint means "not right now, ask again." It is the
device's standard signaling for "I'm busy." For a USB Mass Storage
device, the most common NAK contexts are:

- **CSW (Bulk IN)** while the device is committing the prior
  `WRITE_10` to flash. Can NAK for hundreds of ms to several seconds
  on a sector overwrite that requires a flash erase cycle.
- **CBW data phase (Bulk OUT)** while the device is processing the
  prior command.
- **STATUS phase of control transfers** while the device is still
  processing the SETUP.

### 8.2 NAK retry budgets

Real OS USB stacks retry NAKs for ~30 seconds before giving up. On an
embedded target with no concurrency, that's a long blocking wait —
but a too-small budget breaks legitimate use cases. **Recommended
budgets** (validated against typical consumer USB drives):

| Path                  | Budget                      | Wall time   |
|-----------------------|-----------------------------|-------------|
| Bulk OUT (CBW/data)   | 1000 retries × 5ms          | ~5 seconds  |
| Bulk IN (data/CSW)    | 500 retries × 10ms          | ~5 seconds  |
| Control STATUS phase  | 500 retries × 10ms          | ~5 seconds  |

A 500ms budget on Bulk IN is **not enough**. It works for fresh-page
writes (the device just programs an unprogrammed flash page) but
fails for sector **overwrites**, which require the drive to perform a
read–erase–program cycle on the affected flash erase block. This
takes 1–3 seconds on cheap drives, exceeding 500ms and causing an
intermittent "I/O error when overwriting a specific file" bug that is
hard to diagnose without seeing the NAK ring in telemetry.

### 8.3 STALL is fatal at this layer

A STALL on bulk means the device's endpoint has halted. The standard
recovery is class-level: send a CLEAR_FEATURE(ENDPOINT_HALT) control
transfer to un-halt each affected endpoint, or in the BOT class
specifically, send a **Bulk-Only Mass Storage Reset (BOMSR)** which
resets both bulk endpoints together.

The driver should treat a STALL in bulk OUT or bulk IN as "give up
this transfer, attempt BOMSR." If BOMSR succeeds, the upper SCSI
layer can retry the failed command. If BOMSR itself fails (its
control SETUP NAKs forever), surface an I/O error to the FAT layer.

### 8.4 BUSTIMEOUT vs ERROR vs no TOKDNE

The SIE has three failure modes that look superficially similar:

| Symptom                        | Cause                          |
|--------------------------------|--------------------------------|
| TOKDNE fires, PID = 0          | BUSTIMEOUT — device didn't respond |
| ERROR fires (ERRSTAT.BTOERR)   | Same physical event, different reporting |
| No TOKDNE, no ERROR, BDT OWN=1 | Controller didn't issue the token at all |

The third case is the one to watch for. Causes include:

- `TX_SUSPEND_BUSY` stuck high — the SIE thinks it's mid-transmission
  and won't start a new one. Forces a hardware-level `USB_EN` cycle to
  recover.
- OTG control register cleared — PHY is no longer driving as host.
- INTEN.ATTACHEN never re-armed after a re-init, so the bridge thread
  never gets fed.

When the symptom appears, dump CTL, OTGCTL, OTGSTAT, INTEN, and the
last BDT word-0 to telemetry — that combination disambiguates all
three causes.

---

## 9. SCSI Bulk-Only Transport (BOT)

Standard USB MSC + BOT. The driver implements a single SCSI sequence:

1. **INQUIRY** (6-byte CDB) — confirms the device is responsive.
2. **TEST UNIT READY** (TUR) — loop until ready.
3. **READ CAPACITY (10)** — publishes block_size and block_count.

After that, host READs and WRITEs use **READ_10 / WRITE_10** (10-byte
CDBs), one BOT command per call. Each command is:

1. Host sends **CBW** (Command Block Wrapper, 31 bytes, Bulk OUT).
2. Data phase: Bulk IN or Bulk OUT depending on direction.
3. Host receives **CSW** (Command Status Wrapper, 13 bytes, Bulk IN).

The BOT layer's correctness hinges on three things:

- **Tag matching:** every CBW gets a fresh `dCBWTag`; the corresponding
  CSW must echo it.
- **Toggle preservation:** the bulk EP DATA0/DATA1 toggle must survive
  across CBW → data → CSW boundaries (i.e. it's per-endpoint, not
  per-command).
- **Recovery on data-phase short transfer:** if the device returns less
  data than the CBW requested, drain the residual byte count from the
  CSW's `dCSWDataResidue` field rather than re-issuing the command.

---

## 10. Engineering pitfalls

Distilled from development. Each was a multi-hour debug session.

### 10.1 The MMIO wrapper trap

Wrapping the environment's MMIO primitive (e.g. libdragon's `io_read`)
in a same-signature function — even one the compiler tail-calls —
breaks enumeration. Use macros that expand directly to the primitive.

### 10.2 Per-call vs one-shot init

The temptation to make `init(port)` fully idempotent (skip everything
if already initialized) loses correctness. Re-init must redo the
controller bring-up every call; only thread/handler installation is
one-shot. See §4.2.

### 10.3 USB_EN is VBUS on this platform

On this PHY, toggling `USB_EN` briefly cycles VBUS. This means:

- Bus reset (which sets USB_EN) power-cycles the device — correct, by
  design.
- Calling bus reset *between SETUP retries* power-cycles a device
  that's only partway through booting, restarting the warmup clock.
  Don't do this. Per-transaction recovery (`retry_recovery_soft`)
  must avoid USB_EN.

### 10.4 ATTACH is level, not edge

See §5.2. Mask `ATTACHEN` after the first ATTACH; re-arm it explicitly
in each fresh init.

### 10.5 BDT pong desync after CTL writes

Some `CTL` writes have been observed to silently desync the BDT
EVEN/ODD bank tracker from the SW cursor. The two-step USB_EN release
(see §6.2 step 5) is one workaround; the other is `CTL.ODD_RST` on
quiesce. Always reset the SW pong cursor to Even when the controller
bus-resets.

### 10.6 NAK budgets per layer

A small NAK budget passes light testing and fails real-world overwrite
patterns. See §8.2 — 5 seconds is a reasonable target across all
NAK-tolerant retry sites.

### 10.7 Soft-reset thread orphaning

Any system reboot (full OS re-init, NMI) that doesn't clear the
driver's `s_initted` flag will orphan the bridge thread. The
driver must expose a `notify_reset()` API that the iodev's
`cpu_reset` hook calls before performing the actual reboot. See §5.4.

### 10.8 Telemetry struct is essential

Hardware bugs in this stack are exceptionally hard to diagnose without
a live observable. A small struct at a fixed memory address —
containing interrupt counters, last register snapshots, a ring buffer
of the last N tokens issued (with PID and pong-at-issue) — is the
single highest-leverage thing in the codebase. Multiple bugs that
looked identical from the outside ("disk doesn't mount") were
distinguishable in seconds with the telemetry struct visible in a
memory viewer.

Suggested fields: ISR/attach/tokdne/error counters, last
ISTAT/ERRSTAT/OTGSTAT, last BDT word-0 for the EP0 TX-EVEN slot, the
current phase (a small enum advancing through bring-up steps), an
8-entry ring of `{pong, dir_ep, pid_back, token_type}` for the most
recent tokens, BOMSR attempt/success counters, and the first 8 bytes
of the captured device descriptor.

---

## 11. iodev integration

The driver exposes itself to the FAT layer as a standard iodev:

```c
struct iodev iquesync_iodev = {
    .probe       = iquesync_probe,
    .disk_init   = iquesync_disk_init,
    .disk_read   = iquesync_disk_read,
    .disk_write  = iquesync_disk_write,
    .cpu_reset   = iquesync_cpu_reset,
};
```

`probe` returns success iff running on iQue hardware. `disk_init`
brings up the USB stack and waits for enumeration to complete (~3s
worst case for a slow device). `disk_read/write` translate
512-byte-block requests into BOT WRITE_10/READ_10. `cpu_reset` runs
the soft-reset hook described in §5.4 — invalidate FS mount, clear
driver `s_initted`, then call the OS reboot.

---

## 12. What's deliberately out of scope

- Hub support (single direct-connect device only).
- Low-speed devices (mouse, keyboard).
- Isochronous transfers.
- Hot-replug after enumeration (replug requires a manual "reset disk"
  from the FS layer).
- Sleep / suspend / remote wakeup.
- Detach handling beyond invalidating block geometry.

Adding any of these requires non-trivial additions to the design; do
not assume the existing structure accommodates them.
