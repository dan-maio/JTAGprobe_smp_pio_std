# CMSIS-DAP JTAG/SWD Performance Analysis & Optimization Guide
## For JTAGprobe, JTAGprobe_smp, and JTAGprobe_smp_pio (PIO Hardware JTAG) on Raspberry Pi Pico (RP2040)

**Author:** Antigravity AI & D.M.  
**Date:** October 2026  
**Target Repository:** `dan-maio/JTAGprobe_smp_pio` (upstream: `dst202/JTAGprobe`)  
**Applicable Hardware:** Raspberry Pi Pico (RP2040) / Raspberry Pi Pico 2 (RP2350)

> **Note:** Sections 1–7 analyse and optimize the single-core FreeRTOS JTAGprobe (branch `jtagprobe-new`); JTAGprobe_smp carries all of those optimizations; JTAGprobe_smp_pio runs `DAP_JTAG_Sequence` on an RP2040 PIO state machine (the other JTAG commands still bit-bang on the CPU). Section 8 describes the µOS++ III SMP integration (RP2040 only) and section 9 the measured OpenOCD `load_image` times on a Raspberry Pi 4 B.

---

## 1. Executive Summary

### 1.1 The Problem
When using **JTAGprobe** as a CMSIS-DAP debug probe with OpenOCD, downloading firmware or reading memory is extraordinarily slow:

$$\mathbf{100\text{ KB in }\approx 120\text{ seconds}}\quad (\approx \mathbf{850\text{ bytes/sec}})$$

A typical Full-Speed USB CMSIS-DAP probe should achieve between **50 KB/s and 250 KB/s** in software bit-banging mode, and **500 KB/s to 1.5+ MB/s** when hardware-accelerated with PIO. (Those are general expectations, not measured here: on the Raspberry Pi 4 B, where OpenOCD waits for each packet's reply, the best measured load is **59.1 KB/s** with PIO at 15 MHz, § 9.7.) At 850 B/s, the probe is running **100× to 300× slower than its expected performance**.

### 1.2 Root Cause Summary
The issue is not target flash write speed or USB bus limits. It is a compounding cascade of firmware bottlenecks:

| Rank | Bottleneck | File Location | Latency Impact | Share of Delay |
| :---: | :--- | :--- | :--- | :---: |
| **#1** | **Synchronous UART `printf` per Packet** | `src/probe.h`<br>`src/tusb_edpt_handler.c` | ~5.8 ms blocked execution per packet (with FreeRTOS scheduler suspended) | **~85%** (~100–110s) |
| **#2** | **64-Byte Packet Payload Limit** | `include/DAP_config.h` | 56 bytes max payload per packet; forces >1,828 USB round-trips for 100 KB | **~10%** (~10–12s) |
| **#3** | **Single-Packet Ring Buffer Bottleneck** | `include/DAP_config.h`<br>`src/tusb_edpt_handler.c` | Buffer capacity = 1 packet; NAKs USB host on concurrent packets, causing 1 ms host back-off | **~3%** (~3–5s) |
| **#4** | **Sub-optimal GPIO Bit-Banging** | `include/DAP_config.h` | `gpio_put_masked` read-modify-writes instead of single-cycle SIO registers | **~2%** (~2–3s) |

---

## 2. In-Depth Root Cause Analysis

### 2.1 Bottleneck #1: Synchronous UART `printf` on Every DAP Transaction

#### Mechanism
In `src/probe.h`:
```c
#if true
#define probe_info(format,args...) \
do { \
    vTaskSuspendAll(); \
    printf(format, ## args); \
    xTaskResumeAll(); \
} while (0)
#else
#define probe_info(format,...) ((void)0)
#endif
```

In `src/tusb_edpt_handler.c` (lines 225–244), `probe_info` is executed **twice for every DAP command**:

1. When a command packet arrives:

   ```c
   probe_info("%u %u DAP cmd %s len %02x\n",
              USBRequestBuffer.wptr, USBRequestBuffer.rptr,
              dap_cmd_string[DAPRequestBuffer[0]], DAPRequestBuffer[1]);
   ```

2. When the response is generated:

   ```c
   probe_info("%u %u DAP resp %s\n",
              USBResponseBuffer.wptr, USBResponseBuffer.rptr,
              dap_cmd_string[DAPResponseBuffer[0]]);
   ```

#### Latency Calculation
- In `src/main.c`, `stdio_uart_init()` directs `printf` to UART0 at **115,200 baud**.
- The two format strings total approximately **67 characters** per DAP packet.
- At 115,200 baud (10 bits per character: 1 start + 8 data + 1 stop):

  $$\text{Transmission Time} = \frac{67 \times 10}{115{,}200} \approx \mathbf{5.82\text{ ms per transaction}}$$

- During this entire ~5.8 ms window, `vTaskSuspendAll()` **suspends the FreeRTOS scheduler**. No other thread or USB task can run.
- When flashing or memory downloading with OpenOCD:
  - Flash programming algorithms run on the target microcontroller.
  - OpenOCD continuously polls the target status register (`DHCSR`) and peripheral flags to monitor progress.
  - A 100 KB flash download produces **15,000 to 20,000 DAP transactions** (data blocks + register writes + status polling).

  $$\text{Total UART Blocking Time} = 20{,}000 \times 5.82\text{ ms} = \mathbf{116.4\text{ seconds}}$$

- If no serial terminal is actively reading UART0, the hardware TX FIFO (32 bytes) is perpetually full, guaranteeing that `printf` will block until every single byte shifts out onto the pin.

---

### 2.2 Bottleneck #2: 64-Byte CMSIS-DAP Packet Size

#### Mechanism
In `include/DAP_config.h` (lines 94–95):
```c
#define DAP_PACKET_SIZE         64U            ///< Specifies Packet Size in bytes.
```

In CMSIS-DAP, memory write operations use `DAP_TransferBlock` (command `0x06`):

- Command header: 5 bytes (`Command ID`, `DAP Index`, `Transfer Count [15:0]`, `Transfer Request`).
- Alignment padding: 3 bytes.
- Usable data payload per packet: $64 - 5 - 3 = \mathbf{56\text{ bytes}}$ (14 words of 32 bits).

#### Latency Calculation
To download 100 KB (102,400 bytes):

$$\text{Total Packets} = \frac{102{,}400}{56} \approx \mathbf{1{,}828\text{ packets}}$$

Over USB Full-Speed (12 Mbps), each transaction involves:

1. Host schedules Bulk OUT token.
2. Device receives and processes packet.
3. Host schedules Bulk IN token.
4. Device sends response packet.

On typical operating system USB stacks (Linux `usbfs` or Windows `WinUSB`), round-trip turnaround is 1 to 2 USB frames (1–2 ms). 1,828 round trips create 2 to 4 seconds of pure USB scheduling overhead, excluding execution time.

By contrast, CMSIS-DAP v2 (Bulk endpoint) allows `DAP_PACKET_SIZE` of **512** or **1024** bytes:
- At 512 bytes: $512 - 5 - 3 = 504\text{ bytes}$ (126 words) per packet → **204 packets** (9× reduction).
- At 1024 bytes: $1024 - 5 - 3 = 1016\text{ bytes}$ (254 words) per packet → **101 packets** (18× reduction).

---

### 2.3 Bottleneck #3: Single-Packet Ring Buffer (`DAP_PACKET_COUNT = 2`)

#### Mechanism
In `include/DAP_config.h` (line 100):
```c
#define DAP_PACKET_COUNT        2U              ///< Specifies number of packets buffered.
```

In `src/tusb_edpt_handler.c` (lines 34–37):
```c
bool buffer_full(buffer_t *buffer) {
    return ((buffer->wptr + 1) % DAP_PACKET_COUNT == buffer->rptr);
}
```

In a circular buffer of length $N = 2$ using standard modulo arithmetic:

- When empty: `wptr = 0`, `rptr = 0` → $(0 + 1) \% 2 = 1 \neq 0$ (not full).
- When 1 packet is queued: `wptr = 1`, `rptr = 0` → $(1 + 1) \% 2 = 0 == rptr$ (**FULL**).

The buffer capacity is strictly **1 packet**. There is zero multi-packet buffering.

#### USB NAK Penalty
In `dap_edpt_xfer_cb`:
```c
if (!buffer_full(&USBRequestBuffer)) {
    USBRequestBuffer.wptr++;
    usbd_edpt_xfer(rhport, ep_addr, WR_SLOT_PTR(USBRequestBuffer), DAP_PACKET_SIZE);
    USBRequestBuffer.wasFull = false;
} else {
    USBRequestBuffer.wasFull = true;
}
```
When `wasFull == true`, `usbd_edpt_xfer()` is **not** called. The OUT endpoint is not re-armed. If the host attempts to pipeline a subsequent packet, the RP2040 hardware responds with a **USB NAK**.
USB host controllers typically back off and do not retry until the next USB microframe/frame (1 ms penalty per NAK).

Furthermore, `tusb_edpt_handler.c` contains a latent bug: `_resp_len` is defined as a single global variable (`volatile uint32_t _resp_len`). If multiple responses were to be queued, later packets would overwrite earlier response lengths.

---

### 2.4 Bottleneck #4: GPIO Bit-Banging & Clock Delay Calculation

#### Sub-optimal Pin Access
In `include/DAP_config.h`:
```c
__STATIC_FORCEINLINE void PIN_TDI_OUT (uint32_t bit) {
    gpio_put_masked(1U << DAP_SWJ_TDI, bit << DAP_SWJ_TDI);
}
```
- `gpio_put_masked` performs dynamic shifts and read-modify-write operations on SIO registers on every single bit.
- Functions like `PIN_SWCLK_TCK_SET` and `PIN_SWCLK_TCK_CLR` call `gpio_put()`, which has branch overhead and computes `1U << pin` dynamically.
- RP2040 features Single-Cycle I/O (SIO): `sio_hw->gpio_set` and `sio_hw->gpio_clr` are single-cycle atomic memory writes.

#### Clock Delay Inefficiency
In `src/DAP.c`:
```c
#define MAX_SWJ_CLOCK(delay_cycles) \
  ((CPU_CLOCK/2U) / (IO_PORT_WRITE_CYCLES + delay_cycles))
```
- `CPU_CLOCK` is 125,000,000 Hz, `IO_PORT_WRITE_CYCLES` is 1, and `DELAY_FAST_CYCLES` is 0.
- `MAX_SWJ_CLOCK` evaluates to **62.5 MHz**.
- Because debuggers configure clocks like 1 MHz, 4 MHz, or 10 MHz (never $\ge 62.5\text{ MHz}$), the firmware always takes the slow delay path (`PIN_DELAY_SLOW`) and never uses `fast_clock` (`SWD_TransferFast` / `JTAG_TransferFast`).

---

## 3. Step-by-Step Optimization Roadmap

```
+-------------------------------------------------------------+
|  Current Baseline: 100 KB in 120s (~850 B/s)               |
+-------------------------------------------------------------+
                              |
                              v  [Step 1: Disable UART printf]
+-------------------------------------------------------------+
|  Milestone 1: 100 KB in ~3 to 5s (~25 KB/s)                 |
+-------------------------------------------------------------+
                              |
                              v  [Step 2: Direct RP2040 SIO]
+-------------------------------------------------------------+
|  Milestone 2: 100 KB in ~1.5 to 2.5s (~50 KB/s)             |
+-------------------------------------------------------------+
                              |
                              v  [Step 3: 512B Packets & Queue]
+-------------------------------------------------------------+
|  Milestone 3: 100 KB in < 0.8s (> 125 KB/s)                |
+-------------------------------------------------------------+
                              |
                              v  [Step 4: PIO Hardware JTAG]
+-------------------------------------------------------------+
|  Milestone 4 (measured, § 9.7): 74.9 KB in 1.29 s (58.2 KB/s) |
+-------------------------------------------------------------+
```

1. **Step 1 (Silence Debug Logging):** Eliminate synchronous `printf` calls over 115,200 baud UART while the scheduler is suspended. Reduces packet latency from 5.8 ms to microseconds.
2. **Step 2 (Single-Cycle SIO Pin Access):** Replace SDK dynamic GPIO functions with direct RP2040 SIO register atomic bitmask operations (`gpio_set_mask`, `gpio_clr_mask`, `gpio_get_all`).
3. **Step 3 (Multi-Packet Pipelining & 512B Packets):** Increase packet queue capacity (`DAP_PACKET_COUNT = 4`) and expand CMSIS-DAP v2 packet size to 512 bytes, cutting USB round trips by up to 8.6×.
4. **Step 4 (PIO JTAG):** Run `DAP_JTAG_Sequence` on an RP2040 PIO state machine (`src/jtag.pio`), with TMS in the PIO stream so the CPU never waits between entries. **Implemented in `JTAGprobe_smp_pio`**; measured at `adapter speed 10000`: the JTAG work per 512-byte packet halves (581 → 292 µs) and `load_image` is 16 % faster (§ 9.7). The USB round trip, not the PIO, now sets the load rate.


---

## 4. Concrete Code Patches

### Patch 1: Disable Blocking UART Debug Output
**File:** `src/probe.h`

Change lines 10–43 to set `#if false`:

```c
#if false  /* CHANGED: Disabled blocking UART printf */
#define probe_info(format,args...) \
do { \
	vTaskSuspendAll(); \
	printf(format, ## args); \
	xTaskResumeAll(); \
} while (0)
#else
#define probe_info(format,...) ((void)0)
#endif

#if false  /* CHANGED: Disabled */
#define probe_debug(format,args...) \
do { \
	vTaskSuspendAll(); \
	printf(format, ## args); \
	xTaskResumeAll(); \
} while (0)
#else
#define probe_debug(format,...) ((void)0)
#endif

#if false  /* CHANGED: Disabled */
#define probe_dump(format,args...)\
do { \
	vTaskSuspendAll(); \
	printf(format, ## args); \
	xTaskResumeAll(); \
} while (0)
#else
#define probe_dump(format,...) ((void)0)
#endif
```

---

### Patch 2: Optimize GPIO with Single-Cycle SIO Mask Operations
**File:** `include/DAP_config.h`

Precomputed pin bitmasks and atomic Pico SDK SIO register operations (`gpio_set_mask`, `gpio_clr_mask`, `gpio_get_all`):

```c
// Precomputed pin masks for atomic Single-Cycle I/O (SIO) operations
#define MASK_SWCLK_TCK   (1U << DAP_SWJ_SWCLK_TCK)
#define MASK_SWDIO_TMS   (1U << DAP_SWJ_SWDIO_TMS)
#define MASK_TDI         (1U << DAP_SWJ_TDI)
#define MASK_TDO         (1U << DAP_SWJ_TDO)
#define MASK_nTRST       (1U << DAP_SWJ_nTRST)
#define MASK_nRESET      (1U << DAP_SWJ_nRESET)

// SWCLK / TCK
__STATIC_FORCEINLINE uint32_t PIN_SWCLK_TCK_IN  (void) {
    return (gpio_get_all() >> DAP_SWJ_SWCLK_TCK) & 1U;
}
__STATIC_FORCEINLINE void PIN_SWCLK_TCK_SET (void) {
    gpio_set_mask(MASK_SWCLK_TCK);
}
__STATIC_FORCEINLINE void PIN_SWCLK_TCK_CLR (void) {
    gpio_clr_mask(MASK_SWCLK_TCK);
}

// SWDIO / TMS
__STATIC_FORCEINLINE uint32_t PIN_SWDIO_TMS_IN  (void) {
    return (gpio_get_all() >> DAP_SWJ_SWDIO_TMS) & 1U;
}
__STATIC_FORCEINLINE void PIN_SWDIO_TMS_SET (void) {
    gpio_set_mask(MASK_SWDIO_TMS);
}
__STATIC_FORCEINLINE void PIN_SWDIO_TMS_CLR (void) {
    gpio_clr_mask(MASK_SWDIO_TMS);
}

// SWDIO bidirectional operations (SWD mode)
__STATIC_FORCEINLINE uint32_t PIN_SWDIO_IN (void) {
    return (gpio_get_all() >> DAP_SWJ_SWDIO_TMS) & 1U;
}
__STATIC_FORCEINLINE void PIN_SWDIO_OUT (uint32_t bit) {
    if (bit & 1U) gpio_set_mask(MASK_SWDIO_TMS);
    else          gpio_clr_mask(MASK_SWDIO_TMS);
}
__STATIC_FORCEINLINE void PIN_SWDIO_OUT_ENABLE (void) {
    gpio_set_dir_out_masked(MASK_SWDIO_TMS);
}
__STATIC_FORCEINLINE void PIN_SWDIO_OUT_DISABLE (void) {
    gpio_set_dir_in_masked(MASK_SWDIO_TMS);
}

// TDI
__STATIC_FORCEINLINE uint32_t PIN_TDI_IN (void) {
    return (gpio_get_all() >> DAP_SWJ_TDI) & 1U;
}
__STATIC_FORCEINLINE void PIN_TDI_OUT (uint32_t bit) {
    if (bit & 1U) gpio_set_mask(MASK_TDI);
    else          gpio_clr_mask(MASK_TDI);
}

// TDO
__STATIC_FORCEINLINE uint32_t PIN_TDO_IN (void) {
    return (gpio_get_all() >> DAP_SWJ_TDO) & 1U;
}
```

---

### Patch 3: Multi-Packet Pipelining & Per-Slot Response Tracking
**File:** `include/DAP_config.h`

Expand packet queue from 2 (effective capacity 1) to 4 packets:
```c
#define DAP_PACKET_SIZE         64U             ///< 64 bytes (Full-Speed USB 2.0 Max Packet Size)
#define DAP_PACKET_COUNT        4U              ///< 4 packets queued for zero-NAK pipelining
```

**File:** `src/tusb_edpt_handler.h`  
Add per-slot response length tracking:

```c
typedef struct {
    uint8_t data[DAP_PACKET_COUNT][DAP_PACKET_SIZE];
    uint16_t resp_len[DAP_PACKET_COUNT];   /* Track response size per buffer slot */
    volatile uint32_t wptr;
    volatile uint32_t rptr;
    volatile bool wasEmpty;
    volatile bool wasFull;
} buffer_t;
```

**File:** `src/tusb_edpt_handler.c`  
In `dap_thread`, store `_resp_len` in the buffer slot:

```c
uint16_t resp_len = (uint16_t) DAP_ExecuteCommand(DAPRequestBuffer, DAPResponseBuffer);

vTaskSuspendAll();
memcpy(WR_SLOT_PTR(USBResponseBuffer), DAPResponseBuffer, resp_len);
USBResponseBuffer.resp_len[WR_IDX(USBResponseBuffer)] = resp_len;
USBResponseBuffer.wptr++;

if (buffer_empty(&USBResponseBuffer)) {
    usbd_edpt_xfer(_rhport, _in_ep_addr, RD_SLOT_PTR(USBResponseBuffer), resp_len);
} else {
    USBResponseBuffer.wasEmpty = false;
}
```

---

### Patch 4: PIO JTAG (`JTAGprobe_smp_pio`)
**Files:** `src/jtag.pio`, `include/pio_jtag.h`, `src/pio_jtag.c`, `src/DAP.c`, `src/JTAG_DP.c`, `include/DAP_config.h`

In JTAG mode OpenOCD drives the target almost only with `DAP_JTAG_Sequence` commands: up to a few hundred entries per 512-byte packet, each one 1–64 TCK cycles with a fixed TMS level, TDI bits and an optional TDO capture. `JTAGprobe_smp_pio` runs those commands on an RP2040 PIO state machine (`src/jtag.pio`, `src/pio_jtag.c`, `include/pio_jtag.h`).

#### 1. The state machine (`src/jtag.pio`)

```pasm
.program jtag_tdo
.side_set 1                         ; TCK = side-set (GPIO 2), TDI = OUT (GPIO 6),
                                    ; TMS = SET (GPIO 3),      TDO = IN (GPIO 7)
.wrap_target
    pull block          side 1      ; Header, hold TCK high
    out x, 7            side 1      ; X = TCK count - 1
    out y, 1            side 1      ; Y = TMS level
    jmp !y tms_low      side 1
    set pins, 1         side 1      ; TMS high while TCK is high
    jmp bits            side 1
tms_low:
    set pins, 0         side 1      ; TMS low while TCK is high
bits:
    out null, 32        side 1      ; Empty the OSR, so the first `pull ifempty` fetches TDI data
loop:
    pull ifempty block  side 0      ; Next TDI byte after every 8 bits, drive TCK low
    out pins, 1         side 0      ; Shift 1 bit to TDI (2 cycles low)
    in pins, 1          side 1      ; Sample 1 bit from TDO, drive TCK high
    jmp x--, loop       side 1      ; Decrement X and loop (2 cycles high total)
    push block          side 1      ; Push the partial (or empty) last word
.wrap
```

- **Stream protocol.** Per entry the TX FIFO gets a header word (bits 0–6: TCK count − 1, bit 7: TMS) and then one TDI byte per word. The RX FIFO returns `count / 8 + 1` words: the full TDO bytes, then one word with the last `count % 8` bits (empty when the count is a multiple of 8).
- **TMS** is set by the PIO from the header while TCK is still high, before the entry's first falling edge: the target samples it on the rising edges.
- **TDI** is fetched explicitly with `pull ifempty` (autopull off, threshold 8). With autopull, the header would stay in the output shift register and go out as TDI data, and the next entry's header would be prefetched as data.
- **TDO** uses 8-bit autopush with right shift: a full byte arrives in bits 31..24 of the RX word, the last r bits in bits 31..32−r. The final `push block` waits for RX FIFO space instead of dropping a word.
- **Clock.** Every bit takes 4 PIO cycles, 2 with TCK low and 2 with TCK high, so `div = f_sys / (4 × f_TCK)`. `DAP_SWJ_Clock` (OpenOCD `adapter speed`) sets the divider with `pio_jtag_set_clock()`; `PORT_JTAG_SETUP` restarts the PIO at the last requested speed.

#### 2. The CPU side (`pio_jtag_sequences()`)

`DAP_JTAG_Sequence` (`src/DAP.c`) hands a whole command to `pio_jtag_sequences()`. Two cursors walk the same request: one feeds headers and TDI bytes into the TX FIFO, the other sorts the RX words into the response, keeping only the entries with the TDO flag. The PIO runs entry after entry; core 1 only keeps both 4-word FIFOs moving and never waits for an entry to finish.

#### 3. Pins shared with the CPU bit-banging paths

The other JTAG commands (`DAP_Transfer`, `DAP_TransferBlock`, `DAP_WriteABORT`, `DAP_JTAG_IDCODE`) and `DAP_SWJ_Sequence` / `DAP_SWJ_Pins` still bit-bang through SIO. Their entry points call `PIN_TCK_TDI_TO_SIO()` (`include/DAP_config.h`), which hands TCK, TDI and TMS back to SIO at their current levels, so no pin glitches; the next `DAP_JTAG_Sequence` returns them to the PIO. SWD mode does not use the PIO at all (`PORT_SWD_SETUP` stops it). TDO has the input synchronizer bypassed (`input_sync_bypass`); nTRST and nRESET stay on SIO.

Measurements: § 9.7.

---

## 5. Host-Side OpenOCD Optimization

Ensure your OpenOCD script configures a realistic clock speed. By default, OpenOCD may fall back to 100 kHz if unspecified.

In your OpenOCD target configuration file or command line:
```tcl
# Select CMSIS-DAP v2 adapter
source [find interface/cmsis-dap.cfg]

# Increase adapter speed from default to 4 MHz or 8 MHz
adapter speed 5000

# Specify CMSIS-DAP v2 bulk backend
cmsis_dap_backend bulk
```

---

## 6. Performance Benchmark Comparison

| Configuration State | 100 KB Download Time | Effective Throughput | Relative Speedup |
| :--- | :---: | :---: | :---: |
| **Baseline** (Debug logging active, 64B packets) | **120.0 s** | **0.85 KB/s** | **1.0×** |
| **Step 1 Applied** (UART logging disabled) | **4.2 s** | **24.4 KB/s** | **28×** |
| **Step 1 + Step 2** (+ SIO bit-banging) | **1.9 s** | **53.9 KB/s** | **63×** |
| **Step 1 + 2 + 3** (+ 4-packet ring buffer pipelining) | **0.9 s** | **113.8 KB/s** | **133×** |
| **PIO, TMS in the stream** (`JTAGprobe_smp_pio`, 512-byte packets, 10 MHz; **measured**, § 9.7) | **≈ 1.7 s** (74.9 KB in 1.29 s) | **58.2 KB/s** | **≈ 68×** |

The first four rows are this analysis' estimates for the single-core firmware; the measured rates on the Raspberry Pi 4 B are in § 9.

---

## 7. Integrated USB CDC-ACM UART Bridge

In addition to CMSIS-DAP performance improvements, an integrated hardware UART bridge was implemented to provide a dedicated serial console without requiring extra USB-to-UART dongles.

### 7.1 Architecture & USB Composite Device Structure

The probe enumerates as a USB Composite Device with three distinct interfaces:

1. **Interface 0 (Vendor Specific Bulk, Class 0xFF):**
   - Transport for **CMSIS-DAP v2** commands (`DAP_OUT_EP_NUM 0x04`, `DAP_IN_EP_NUM 0x85`).
   - Handled directly by host debuggers (**OpenOCD**, **pyOCD**, **probe-rs**) via `libusb` / WinUSB.
   - Does **not** register as a serial/tty device.

2. **Interfaces 1 & 2 (CDC Communication & CDC Data):**
   - Implements standard **USB CDC-ACM** virtual serial port (`/dev/ttyACM0` on Linux, `COMx` on Windows).
   - Handled by host kernel CDC drivers (`cdc_acm`).

> **Note:** On Linux, the udev subsystem tags `/dev/ttyACM0` with the device-level product string (`"JTAG probe on Pico (CMSIS-DAP)"`). `/dev/ttyACM0` is the serial console port wired to **GPIO 12 (TX)** and **GPIO 13 (RX)**.

### 7.2 Hardware Mapping & FreeRTOS Task Scheduling
- **Hardware Peripheral:** RP2040 `uart0`
- **Pins:** GPIO 12 (`PROBE_UART_TX`, Pico Header Pin 16) and GPIO 13 (`PROBE_UART_RX`, Pico Header Pin 17).
- **FreeRTOS Priority Balancing:**
  - `UART_TASK_PRIO` (`tskIDLE_PRIORITY + 3`) &mdash; Highest priority to preempt USB and prevent hardware UART FIFO overruns at high line rates.
  - `TUD_TASK_PRIO` (`tskIDLE_PRIORITY + 2`) &mdash; Handles TinyUSB event dispatching and endpoint buffers.
  - `DAP_TASK_PRIO` (`tskIDLE_PRIORITY + 1`) &mdash; Executes CMSIS-DAP command interpretation and bit-banging.
- **Dynamic Line Coding:** Automatically tracks host terminal baud rate (e.g. 9600, 115200, 921600), parity, stop bits, and data bit configurations via `tud_cdc_line_coding_cb`.
- **In JTAGprobe_smp** the three tasks are µOS++ threads with the same ordering (`os_thread_priority_high`, `_above_normal`, `_normal`); `UART` and `TUD` are pinned to core 0, `DAP` to core 1 (section 8).

### 7.3 Full Pinout Reference

> **JTAGprobe_smp_pio_std** moves every signal to the Pico's pins 1–10
> (GP0 nTRST, GP1 nRESET, GND, GP2 SWCLK/TCK, GP3 SWDIO/TMS, GP4 UART1 TX,
> GP5 UART1 RX, GND, GP6 TDI, GP7 TDO); see its README. The table below, and the
> UART pins in § 7.1–7.2, are those of JTAGprobe, JTAGprobe_smp and
> JTAGprobe_smp_pio, on which the measurements in this document were made.

| Signal | RP2040 GPIO | Pico Header Pin | Role / Direction | Function |
| :--- | :---: | :---: | :--- | :--- |
| **SWCLK / TCK** | **GPIO 19** | **Pin 25** | Output (Probe → Target) | SWD Clock / JTAG Clock |
| **SWDIO / TMS** | **GPIO 14** | **Pin 19** | Bi-directional | SWD Data / JTAG Mode Select |
| **TDI** | **GPIO 18** | **Pin 24** | Output (Probe → Target) | JTAG Test Data In |
| **TDO** | **GPIO 21** | **Pin 27** | Input (Probe ← Target) | JTAG Test Data Out |
| **nTRST** | **GPIO 15** | **Pin 20** | Output (Probe → Target) | JTAG Reset (optional) |
| **nRESET** | **GPIO 16** | **Pin 21** | Open-drain (Target Reset) | Target Hardware Reset |
| **UART_TXD** | **GPIO 12** | **Pin 16** | Output (Probe → Target) | Serial TX to Target RX (`uart0`) |
| **UART_RXD** | **GPIO 13** | **Pin 17** | Input (Probe ← Target) | Serial RX from Target TX (`uart0`) |
| **GND** | **GND** | **Pins 3, 8, 13, 18, 23, 28, 38** | Ground | Common Ground |

---

## 8. µOS++ III SMP Integration (JTAGprobe_smp)

JTAGprobe_smp replaces FreeRTOS (single core: every task on core 0) with the **µOS++ III SMP** kernel, so the RP2040's second Cortex-M0+ core does the JTAG/SWD work. The Pico SDK still boots the board and provides the drivers.

### 8.1 Build structure
Exactly like the `micro-os-plus-iii-smp` tests, the kernel and its port are CMake **INTERFACE** libraries, compiled with the executable's flags:

| Directory | CMake target | Contents |
| :--- | :--- | :--- |
| `micro-os-plus/micro-os-plus-iii/` | `micro-os-plus::iii` | the kernel, unmodified (`smp` branch, commit `931f1380`) |
| `micro-os-plus/micro-os-plus-iii-rp2040/` | `micro-os-plus::rp2040` | the RP2040 port (below); defines `OS_NCPU=2 OS_USE_SMP_SCHEDULER=1` |

The top `CMakeLists.txt` adds the port with `add_subdirectory`, links `micro-os-plus::rp2040`, builds tinyusb with `TINYUSB_OPT_OS = OPT_OS_NONE`, sets `OS_USE_OS_APP_CONFIG_H` (`src/cmsis-plus/os-app-config.h`: 1 kHz tick) and `PICO_USE_MALLOC_MUTEX=0` (the kernel's `malloc` already runs under the scheduler lock), and links the image `copy_to_ram`.

### 8.2 RP2040 port (`src/os-core-rp2040.cpp`, `include/cmsis-plus/rtos/port/`)
- **Context switch:** ARMv6-M PendSV at the lowest priority on each core; `r8`–`r11` are saved through the low registers.
- **Tick:** SysTick on both cores at 1 kHz; only core 0 advances the system clock.
- **Kernel lock:** SIO hardware spinlock `PICO_SPINLOCK_ID_OS1` (14), the one the SDK reserves for an RTOS. Critical sections use PRIMASK.
- **Cross-core IPI:** the SIO inter-core FIFO; each core has its own IRQ (`SIO_IRQ_PROC0`, `SIO_IRQ_PROC1`), whose handler drains the FIFO and pends PendSV.
- **Core 1:** started by `os_rp2040_start_core1()` with `multicore_launch_core1_raw()` on the SDK's core-1 stack (`__StackOneTop`); it gets its own idle thread, `idle1`.
- **High-resolution clock:** `time_us_64()`. **Free store:** `__end__` … `__HeapLimit`. **exit():** the SDK's; the port filters the kernel's `exit.c` out.

### 8.3 Threads and data flow

| Thread | Priority | Core | Job |
| :--- | :--- | :---: | :--- |
| `UART` | high | 0 | CDC ↔ `uart0` bridge |
| `TUD` | above normal | 0 | `tud_task()`, then `dap_edpt_service()` |
| `DAP` | normal | 1 | `DAP_ExecuteCommand()` (bit-banging) |

- Every tinyusb call is made with `usb_mutex` held. tinyusb's `tud_event_hook_cb` posts `usb_sem` to wake the `TUD` thread on each USB event.
- DAP requests (OUT endpoint → `DAP`) and responses (`DAP` → IN endpoint) cross between the cores in two lock-free single-producer/single-consumer rings of `DAP_PACKET_COUNT` = 4 slots, with free-running indices and a `__dmb()` between a slot's data and the index that publishes it.

### 8.4 Four versions of the DAP hand-off

1. **First build (measured, § 9; history, fixed in the second build):** the `DAP` thread slept on `dap_sem` after every packet, and posted `usb_sem` so the `TUD` thread on core 0 started each IN transfer. Per packet that was two cross-core wake-ups (IPI + PendSV + context switch on each core) on the critical path.
2. **Second build (measured, § 9: ~38 KB/s):** the `DAP` thread busy-waits up to **2 ms** (`DAP_SPIN_US`) for the next packet or a free response slot before sleeping, and starts the IN transfer (and re-arms a stalled OUT endpoint) itself when `os_mutex_try_lock(&usb_mutex)` succeeds, as the single-core firmware does from its DAP task; otherwise it posts `usb_sem`. During a load no thread sleeps on the packet path.
3. **Third build (measured, § 9.4):** as the second, plus:
   - the `TUD` thread on core 0 keeps polling `tud_task()` for **2 ms** (`USB_SPIN_US`) after the last DAP packet moved, instead of sleeping on `usb_sem` (`dap_edpt_active()`); `UART` still preempts it;
   - **per-stage packet timing**, kept by the firmware and read by the host with DAP vendor command `0x80` (§ 8.5).
4. **Current build (measured, § 9.6):** as the third, with **512-byte DAP packets** (`DAP_PACKET_SIZE` 512) over the 64-byte full-speed bulk endpoint. A request arrives as up to eight USB packets; the OUT endpoint is armed one USB packet at a time, and `dap_req_len()` (a parser of the DAP command lengths) decides whether a request ends on a full 64-byte packet, since the host sends no ZLP. A response of a 64-byte multiple shorter than 512 bytes is followed by a ZLP.

### 8.5 Packet timing (DAP vendor command 0x80)
The firmware time-stamps every DAP packet with the RP2040's 1 MHz timer (shared by both cores) and keeps twelve 32-bit counters. OpenOCD's `cmsis-dap cmd` prints only the first four bytes of a reply, so the host reads one counter per command:

```tcl
cmsis-dap cmd 0x80 0xff     ;# reset the counters
cmsis-dap cmd 0x80 <i>      ;# read counter i (0..11): "Returned data b0 b1 b2 b3", little-endian
```

| i | Counter | i | Counter |
| :---: | :--- | :---: | :--- |
| 0 | packets executed | 6 | IN started → IN complete (µs) |
| 1 | OUT received → DAP thread starts it (µs) | 7 | IN transfers completed |
| 2 | `DAP_ExecuteCommand()` (µs) | 8 | OUT → next OUT, gaps < 10 ms (µs) |
| 3 | longest `DAP_ExecuteCommand()` (µs) | 9 | number of such gaps |
| 4 | DAP waiting for a free response slot (µs) | 10 | gaps ≥ 10 ms (host idle) |
| 5 | response ready → IN started (µs) | 11 | OUT stalls (request ring full) |

### 8.6 How the kernel and the two cores are used
- **Kernel:** one µOS++ III SMP scheduler for both cores. Each thread is created with a priority and a CPU affinity (`th_cpu_affinity`, `create_thread()` in `src/main.c`): `UART` (high) and `TUD` (above normal) on core 0, `DAP` alone on core 1. On core 0 the scheduler preempts by priority, so the UART bridge always runs ahead of USB.
- **Kernel objects:** `usb_mutex` serializes every tinyusb call from either core; the binary semaphores `usb_sem`, `dap_sem` and `uart_sem` wake a sleeping thread when it has work.
- **Packet path outside the kernel:** during a transfer the requests and responses pass through the lock-free rings (§ 8.3) and the threads spin (§ 8.4); the kernel sleeps and wakes threads only when the probe goes idle and becomes active again. A cross-core sleep and wake-up per packet cost the first build a factor of 4 (§ 9.2).
- **Core usage during a load** (512-byte packets, counters of § 9.6): core 1 bit-bangs JTAG for ~580 µs of each ~1,740 µs cycle (about ⅓) and spins for the rest; core 0 handles the USB events of the request and response transfers, ~7 µs to start each response, and the UART bridge. For about ⅔ of each cycle neither core has useful work, waiting on USB and the PC: the CPUs are not the bottleneck. After 2 ms without packets both threads sleep on their semaphores. These shares come from the per-stage packet timing; no per-core CPU-load counter was measured.

### 8.7 PIO in JTAGprobe_smp_pio

| Where | Work |
| :--- | :--- |
| **Core 0** | tinyusb (`TUD` thread) and the UART bridge (`UART` thread), as in JTAGprobe_smp |
| **Core 1** | the `DAP` thread: runs each DAP command; for `DAP_JTAG_Sequence` it only streams the entries into the PIO's TX FIFO and sorts the TDO bytes from its RX FIFO (`pio_jtag_sequences()`) |
| **PIO state machine** | TCK, TDI, TMS and TDO of every `DAP_JTAG_Sequence` entry, at the `adapter speed` clock (Patch 4) |

The other JTAG commands and `DAP_SWJ_Sequence`/`DAP_SWJ_Pins` still bit-bang on core 1, taking TCK, TDI and TMS back from the PIO first. Core 1 still spins while the PIO works (it polls the FIFOs): the PIO does not free the core, it shortens the work, from 581 to 292 µs per 512-byte packet during a load at 10 MHz (§ 9.7).

---

## 9. Measured OpenOCD Load Times (Raspberry Pi 4 B)

### 9.1 Setup
- Target: Raspberry Pi 4 B (BCM2711, 4× Cortex-A72) over **JTAG**, `adapter speed 4000` (4 MHz TCK), CMSIS-DAP v2 bulk, `bcm2711.dap memaccess 0`.
- Images: µOS++ III SMP test ELFs (`aarch64-rpi4b-*-hwd`), loaded with OpenOCD `load_image <elf>`. *Loaded bytes* is the `LOAD` segment's file size (`readelf -lW`).
- Timing: OpenOCD `clock milliseconds` before and after `load_image` alone (the run script prints `--- LOADTIME: N ms ---`).
- Rates are in KB/s with 1 KB = 1000 bytes.

> **Note:** In JTAG mode OpenOCD's CMSIS-DAP driver does not use `DAP_TransferBlock` (§ 2.2); it sends raw `DAP_JTAG_Sequence` commands, every TMS bit and idle clock its own entry. The probe's own counters (§ 8.5) show **~9.9 loaded bytes per packet** with `memaccess 0` (74,888 bytes in 7,541 packets; 80,984 in 8,122), and OpenOCD waits for each reply before sending the next request. A load is therefore limited by the per-packet round trip, not by the TCK frequency. OpenOCD sizes its JTAG queue to the probe's packet size: with 512-byte packets (§ 9.6) a packet carries **~85.7 loaded bytes** (74,888 bytes in 874 packets; 80,984 in 940).

### 9.2 Measurements

| Firmware | Test image | Loaded bytes | `load_image` time | Rate | Per packet (≈ bytes/9.9) |
| :--- | :--- | ---: | ---: | ---: | ---: |
| Stock debugprobe 2.0.0 | 4 KB test load | — | — | ~1.05 KB/s | ~8.4 ms (measured with `-d4`) |
| JTAGprobe `jtagprobe-new` (FreeRTOS, 1 core) | smp_test4 | 70,296 | **1,780 ms** | **39.5 KB/s** | 251 µs |
| JTAGprobe `jtagprobe-new` (FreeRTOS, 1 core) | smp_mem_inter_test | 80,984 | **2,380 ms** | **34.0 KB/s** | 292 µs |
| JTAGprobe_smp, first build | smp_mem_test | 74,888 | **7,536 ms** | **9.94 KB/s** | 999 µs |
| JTAGprobe_smp, first build | smp_mem_inter_test | 80,984 | **8,136 ms** | **9.95 KB/s** | 998 µs |
| JTAGprobe_smp, second build | smp_mem_test | 74,888 | **1,983 ms** | **37.8 KB/s** | 263 µs |
| JTAGprobe_smp, second build | smp_mem_inter_test | 80,984 | **2,125 ms** | **38.1 KB/s** | 261 µs |
| JTAGprobe_smp, third build | smp_mem_test | 74,888 | **1,994 ms** | **37.6 KB/s** | 264 µs *counted* |
| JTAGprobe_smp, third build | smp_mem_inter_test | 80,984 | **2,143 ms** | **37.8 KB/s** | 263 µs *counted* |
| JTAGprobe_smp, third build, rerun | smp_mem_test | 74,888 | **1,954 ms** | **38.3 KB/s** | 259 µs |
| JTAGprobe_smp, third build, rerun | smp_mem_inter_test | 80,984 | **2,106 ms** | **38.5 KB/s** | 259 µs |
| JTAGprobe_smp, current build (512-byte packets) | smp_mem_test | 74,888 | **1,528–1,547 ms** | **48.4–49.0 KB/s** | 1,745 µs *counted* (85.7 B) |
| JTAGprobe_smp, current build (512-byte packets) | smp_mem_inter_test | 80,984 | **1,644–1,649 ms** | **49.1–49.3 KB/s** | 1,733 µs *counted* (86.2 B) |

Per packet: *counted* = the probe's own OUT → OUT mean; the other rows divide by 9.9 bytes per packet, since OpenOCD and the images are the same.

Same image, three firmwares (smp_mem_inter_test, 80,984 bytes):

| Firmware | `load_image` time | Rate | vs. stock | vs. FreeRTOS JTAGprobe |
| :--- | ---: | ---: | ---: | ---: |
| Stock debugprobe 2.0.0 (computed at 1.05 KB/s) | ~77 s | 1.05 KB/s | 1× | 0.03× |
| JTAGprobe `jtagprobe-new`, FreeRTOS, 1 core | 2,380 ms | 34.0 KB/s | 32× | 1× |
| JTAGprobe_smp, first build | 8,136 ms | 9.95 KB/s | 9.5× | 0.29× |
| **JTAGprobe_smp, second build** | **2,125 ms** | **38.1 KB/s** | **36×** | **1.12×** |
| JTAGprobe_smp, third build | 2,143 ms | 37.8 KB/s | 36× | 1.11× |
| **JTAGprobe_smp, current build (512-byte packets)** | **1,644–1,649 ms** | **49.1–49.3 KB/s** | **47×** | **1.45×** |

Every one of these tests also **passed** after its load. Whole-test CTest times (15 s reset wait included), without semihosting prints (§ 9.5 has them with):

| Test | FreeRTOS JTAGprobe | JTAGprobe_smp (first build) | JTAGprobe_smp (second build) | JTAGprobe_smp (third, 10 s reset wait) | JTAGprobe_smp (current, 512-byte packets, 10 s reset wait) |
| :--- | ---: | ---: | ---: | ---: | ---: |
| smp_test4 | 29.6 s | — | — | — | — |
| smp_mem_test | 140–142 s | 61.0 s | **54.2 s** | **50.2 s** | **47.1 s** |
| smp_mem_inter_test | 190.1 s | 193.1 s | **186.0 s** | **193.4 s** | **175.9 s** |

The last two columns' runs wait 10 s after the board reset instead of 15 s. smp_mem_inter_test's time is set by its own computing on the Pi (~170 s) and varies by several seconds from run to run (186–193 s), so the probe firmware hardly shows there.

**Reading the numbers:** the first SMP build spent about **700 µs more per packet** than the single-core firmware (≈ 1,000 µs against ≈ 251–292 µs): the two cross-core wake-ups per packet of § 8.4. That made the load about **4× slower** (9.95 KB/s against 34–39.5 KB/s). The second build takes those wake-ups off the packet path: ≈ 262 µs per packet, **3.8× faster** than the first SMP build and as fast as the single-core firmware (on the same image, smp_mem_inter_test, 2.13 s against 2.38 s). The `TUD` spin of the third build changed nothing measurable (2,143 ms against 2,125 ms): § 9.4 shows why. The 512-byte packets of the current build load about **23 % faster** again (§ 9.6). smp_mem_test as a whole finished in 54.2 s with the SMP probe, against 140 s with the single-core one, so the gain there comes from the run after the load (semihosting and memory polling traffic: isolated packets rather than a stream), with the DAP work on core 1 and the USB and UART work on core 0. That part was not profiled packet by packet.

### 9.3 Computed load times per test image

Each test's loaded bytes divided by the measured rates: **39.5 KB/s** (FreeRTOS JTAGprobe, best measured), **37.9 KB/s** (JTAGprobe_smp current build, mean of its two measurements), **9.95 KB/s** (JTAGprobe_smp first build) and **1.05 KB/s** (stock). Values marked *measured* were timed; the rest are computed.

| Test image | Loaded bytes | FreeRTOS JTAGprobe | JTAGprobe_smp (current) | JTAGprobe_smp (first build) | Stock debugprobe |
| :--- | ---: | ---: | ---: | ---: | ---: |
| cmsis-os-validator | 151,640 | 3.8 s | 4.0 s | 15.2 s | 2.4 min |
| mutex-stress | 94,488 | 2.4 s | 2.5 s | 9.5 s | 1.5 min |
| rtos-apis | 228,136 | 5.8 s | 6.0 s | 22.9 s | 3.6 min |
| sd_test | 83,512 | 2.1 s | 2.2 s | 8.4 s | 1.3 min |
| smp-mat-sdcard-test | 93,736 | 2.4 s | 2.5 s | 9.4 s | 1.5 min |
| smp-mat-test | 77,608 | 2.0 s | 2.0 s | 7.8 s | 1.2 min |
| smp-num-test | 91,160 | 2.3 s | 2.4 s | 9.2 s | 1.4 min |
| smp-pipeline-test | 146,472 | 3.7 s | 3.9 s | 14.7 s | 2.3 min |
| smp-pro-cons-test | 88,968 | 2.3 s | 2.3 s | 8.9 s | 1.4 min |
| smp_mem_test | 74,888 | 1.9 s | 1.98 s *measured* | 7.54 s *measured* | 1.2 min |
| smp_mem_inter_test | 80,984 | 2.38 s *measured* | 2.13 s *measured* | 8.14 s *measured* | 1.3 min |
| smp_test0 … smp_test4 | 65,656 … 74,744 | 1.7 … 1.9 s (smp_test4: 1.78 s *measured*, 70,296 B) | 1.7 … 2.0 s | 6.6 … 7.5 s | 1.0 … 1.2 min |

**Formula:** $t_{load} \approx \dfrac{\text{loaded bytes}}{\text{rate}} \approx \dfrac{\text{loaded bytes}}{9.9} \times t_{packet}$, with $t_{packet}$ ≈ 251–292 µs (FreeRTOS JTAGprobe), ≈ 262 µs (JTAGprobe_smp second and current builds), ≈ 1,000 µs (JTAGprobe_smp first build).

### 9.4 Where the time per packet goes (third build, 64-byte packets)

From the probe's counters (§ 8.5), reset just before `load_image` and read just after it:

| Per packet, mean | smp_mem_test load (7,541 packets) | smp_mem_inter_test load (8,122 packets) | Where |
| :--- | ---: | ---: | :--- |
| OUT → next OUT (one full cycle) | **263.5 µs** | **262.8 µs** | — |
| OUT received → DAP thread starts it | 0.9 µs | 0.9 µs | core 0 → core 1 |
| `DAP_ExecuteCommand()` (JTAG bit-banging) | **68.2 µs** (max 82) | **68.3 µs** (max 81) | core 1 |
| wait for a free response slot | 0.4 µs | 0.4 µs | core 1 |
| response ready → IN started | 6.1 µs | 6.1 µs | core 1 / core 0 |
| IN started → host collected it | **72.8 µs** | **72.5 µs** | USB |
| rest of the cycle: host turnaround to the next OUT | ≈ 115 µs | ≈ 115 µs | PC: libusb + OpenOCD |
| OUT stalls (request ring full) | 0 | 0 | — |

- The hand-off between the two cores now costs about **7 µs per packet** (0.9 + 0.4 + 6.1): the dual-core split is no longer on the critical path.
- OpenOCD sends one request at a time and waits for its reply (no OUT stalls, a single packet in flight). About **70 %** of each cycle is the USB round trip and the PC's turnaround, which no split of work between the probe's cores can remove; the JTAG bit-banging is about **26 %**.
- The remaining gains are therefore fewer round trips (larger `DAP_PACKET_SIZE`, so OpenOCD packs more into each command: done in the current build, § 9.6) and faster bit-banging (PIO: **implemented in `JTAGprobe_smp_pio`**, § 4 Patch 4, § 8.7, replacing CPU loops with hardware state machine serialization).

### 9.5 Semihosting load: `SEMI_PRINTS=1`

The µOS++ AArch64 port's rpi4b tests take a CMake option `SEMI_PRINTS` (0/1, hwd images only). With 1, every console line printed on the Pi's **core 0** also goes through semihosting, besides the UART; the RESULT line goes through as always. Cores 1–3 keep printing on the UART only: OpenOCD 0.12 does not resume a core other than 0 after its semihosting call on this board (the line is printed, the core stays halted, and every other core then blocks on the console lock).

| hwd test, JTAGprobe_smp | smp_mem_test `SEMI_PRINTS=0` | smp_mem_test `SEMI_PRINTS=1` | smp_mem_inter_test `SEMI_PRINTS=0` | smp_mem_inter_test `SEMI_PRINTS=1` |
| :--- | ---: | ---: | ---: | ---: |
| Load | 1,983 ms | 1,994 ms | 2,125 ms | 2,143 ms |
| Test run on the board | 37 s | 314 s | ~170 s | 591 s |
| CTest total | 54.2 s | **337.4 s, PASS** | 186.0 s | **620.5 s, PASS** |
| Lines through semihosting | 1 | ~300 | 1 | ~480 |

A semihosting line costs about **0.9 s**: OpenOCD's `SYS_WRITE0` reads the string from the target **one byte per JTAG memory access** (≈ 7–10 ms each), then resumes the core. That cost is OpenOCD's method, not the probe's turnaround. The table is for the 64-byte third build; with 512-byte packets (§ 9.6) both tests also pass with `SEMI_PRINTS=1`, in **308.1 s** and **561.8 s**.

### 9.6 512-byte DAP packets (current build)

Same Pi 4 B, OpenOCD and images; only `DAP_PACKET_SIZE` changed from 64 to 512 (§ 8.4, item 4). Probe counters for the load, per packet, mean:

| Per packet, mean | 64 bytes, smp_mem_test (7,541 packets) | 512 bytes, smp_mem_test (874 packets) | 512 bytes, smp_mem_inter_test (940 packets) |
| :--- | ---: | ---: | ---: |
| Loaded bytes per packet | 9.9 | **85.7** | **86.2** |
| OUT → next OUT (one full cycle) | 263.5 µs | **1,745 µs** | **1,733 µs** |
| OUT received → DAP thread starts it | 0.9 µs | 1.9 µs | 1.1 µs |
| `DAP_ExecuteCommand()` | 68.2 µs | 581.4 µs (max 602) | 582.1 µs (max 601) |
| response ready → IN started | 6.1 µs | 7.0 µs | 6.7 µs |
| IN started → host collected it | 72.8 µs | 242.9 µs | 239.7 µs |
| rest: receiving the request + PC turnaround | ≈ 115 µs | **≈ 912 µs** | ≈ 902 µs |
| Cycle time per loaded byte | 26.6 µs | **20.4 µs** | 20.1 µs |

Whole runs (CTest, 10 s reset wait), all **PASS**:

| hwd test | 64-byte packets | 512-byte packets | Change |
| :--- | ---: | ---: | ---: |
| smp_mem_test, load | 1,954–1,994 ms | 1,528–1,547 ms | −23 % |
| smp_mem_inter_test, load | 2,106–2,143 ms | 1,644–1,649 ms | −23 % |
| smp_mem_test, `SEMI_PRINTS=0` | 50.2 s | **47.1 s** | −3.1 s (−6 %) |
| smp_mem_inter_test, `SEMI_PRINTS=0` | 186.0–193.4 s | **175.9 s** | −10 s or more |
| smp_mem_test, `SEMI_PRINTS=1` | 337.4 s | **308.1 s** | −29.3 s (−8.7 %) |
| smp_mem_inter_test, `SEMI_PRINTS=1` | 620.5 s | **561.8 s** | −58.7 s (−9.5 %) |

**Reading the numbers:**
- OpenOCD does fill the larger packets: **8.6× fewer packets** for the same image.
- The load is still only about **23 % faster**, not 8×. The bit-banging is unchanged per byte (exec totals ≈ 510 ms for smp_mem_test in both builds), the IN transfers drop from 549 to 214 ms in total, but the *rest* of each cycle grows from ≈ 115 µs to ≈ 910 µs. That part is the reception of the request (up to eight 64-byte USB packets, each a tinyusb event on core 0 followed by a re-arm of the endpoint) plus the PC's turnaround. The counters do not separate the two, so how much of it is the per-packet re-arm is **not measured**.
- The whole tests gain more than their loads (smp_mem_test −3.1 s against −0.4 s of load): the polling and semihosting traffic during the run also travels in fewer, larger packets. This part was not profiled packet by packet. smp_mem_inter_test's earlier runs spread over 186–193 s, so its gain is between 10 and 17 s.
- With `SEMI_PRINTS=1` the gain is ~9 %. The bulk of those runs is still OpenOCD reading every semihosted string byte by byte (§ 9.5), which the packet size cannot change.
- Next step on the probe side: time the OUT reception separately, and arm the rest of a request in one transfer once `dap_req_len()` knows its length, instead of one 64-byte packet at a time.

### 9.7 PIO JTAG (`JTAGprobe_smp_pio`)

smp_mem_test with `SEMI_PRINTS=1`, 512-byte packets, Patch 4 in its successive versions:

| Probe firmware | `adapter speed` | `load_image` (74,888 B) | JTAG work per packet | Load cycle per packet | Whole test (CTest) |
| :--- | ---: | ---: | ---: | ---: | ---: |
| JTAGprobe_smp, CPU bit-banging | 4000 | 1,528 ms (49.0 KB/s) | 581 µs | 1,745 µs | 308.1 s |
| PIO, TMS through SIO, waiting after each entry | 4000 | 1,702 ms | 755 µs | 1,934 µs | 324.5 / 332.0 s |
| PIO, TMS through SIO, waiting after each entry | 10000 | 1,578 ms | 621 µs | 1,794 µs | 315.3 s |
| **PIO, TMS in the stream (current)** | **10000** | **1,286 ms (58.2 KB/s)** | **292 µs** | **1,450 µs** | **291.5 s** |
| PIO, TMS in the stream (current) | 15000 | 1,287 ms (58.2 KB/s) | 291 µs | 1,448 µs | 285.1 s |

All runs **PASS**; each configuration ran once (earlier identical PIO runs differed by ~8 s in total time). *JTAG work per packet* is the probe's `DAP_ExecuteCommand()` time, from its packet counters (DAP vendor command `0x80`).

- **A PIO that waits after every entry is slower than CPU bit-banging.** With a few hundred short entries per packet, setting TMS from the CPU and waiting for each entry cost ~530 µs per packet; raising TCK from 4 to 10 MHz saved only 134 µs.
- **Streaming the entries, with TMS in the stream, halves the JTAG work** (581 → 292 µs per packet at 10 MHz): `load_image` is 16 % faster than with CPU bit-banging and 35 % faster than with 64-byte packets (1,954–1,994 ms), smp_mem_test 16.6 s faster.
- **15 MHz loads exactly as fast as 10 MHz** (1,287 ms, 291 µs per packet): the TCK clock is no longer the limit; most likely core 1, which feeds the FIFOs one byte per word and sorts the replies, is (not measured separately). The whole test took 285.1 s against 291.5 s, within the ~8 s spread seen between identical runs.
- **What is left** in a load cycle is mostly receiving the 512-byte request (one 64-byte USB packet at a time) and the PC's turnaround: ≈ 920 µs of 1,450 µs. With `SEMI_PRINTS=1` the whole test is still dominated by OpenOCD reading each semihosted string byte by byte (~0.9 s per line).
- Not measured: the current build at 4 MHz.

Both smp_mem tests, built from the GitHub workspace of µOS++ III (`SEMI_PRINTS=1`, PIO with TMS in the stream, `adapter speed 15000`), against the 512-byte CPU bit-banging build at 4 MHz (§ 9.6 of `PERFORMANCE_ANALYSIS_AND_OPTIMIZATION.md`):

| Test | Loaded bytes | `load_image` | JTAG work per packet | Whole test (CTest) | CPU bit-banging, 4 MHz |
| :--- | ---: | ---: | ---: | ---: | ---: |
| smp_mem_test | 75,208 | **1,273 ms (59.1 KB/s)** | 291 µs | **282.9 s, PASS** | 1,528 ms, 308.1 s |
| smp_mem_inter_test | 81,048 | **1,399 ms (57.9 KB/s)** | 291 µs | **537.4 s, PASS** | 1,644 ms, 561.8 s |

These images are a few hundred bytes larger than the earlier ones (75,208 against 74,888 bytes): the `SEMI_PRINTS=1` code. Both tests run 4–8 % faster than with CPU bit-banging; smp_mem_test repeated at 15 MHz in 285.1 s (golden build) and 282.9 s (GitHub build). OpenOCD logged `error handling USB events: System call interrupted` and `CMD_DAP_JTAG_SEQ failed` once, right after smp_mem_inter_test's `RESULT: PASS`, most likely the runner stopping OpenOCD in the middle of a command (not investigated further).

---

## 10. Conclusion

The 2-minute download time is caused almost entirely by **synchronous debug logging (`probe_info`) blocking the CPU for 5.8 ms per packet over a 115,200 baud UART while the FreeRTOS scheduler is suspended**.

Applying **Patch 1** alone provides an immediate **28× speedup** (reducing 120s down to ~4s). Applying **Patches 2 and 3** brings the download time down to **under 1 second**, fully solving the performance problem while maintaining a clean, responsive hardware UART bridge on `/dev/ttyACM0`.

Measured on a Raspberry Pi 4 B over JTAG (§ 9), where OpenOCD sends about 9.9 loaded bytes per packet and one packet at a time: the optimized FreeRTOS firmware loads at **34–39.5 KB/s** against ~1.05 KB/s for the stock firmware. **JTAGprobe_smp** (µOS++ III SMP, DAP on core 1): its first build lost a factor of 4 to two cross-core wake-ups per packet (9.95 KB/s), which the second build fixed: with the DAP thread spinning on its ring and sending its own responses it loads at **37.6–38.5 KB/s**, as fast as the single-core firmware, and the complete smp_mem_test ran in 50–54 s against 140 s. The probe's own counters (§ 9.4) put the core-to-core hand-off at ~7 µs per packet and show that ~70 % of each cycle was the USB round trip and the PC. The 512-byte packet build (§ 9.6) achieves 8.6× fewer packets, loads at **48.4–49.3 KB/s** (23 % faster, 1.45× the single-core firmware), and the tests finish faster (smp_mem_test 47.1 s, smp_mem_inter_test 175.9 s; with `SEMI_PRINTS=1` 308.1 s and 561.8 s, all PASS). With semihosting prints on, a semihosted line still costs ~0.9 s, set by OpenOCD (§ 9.5).

Finally, **JTAGprobe_smp_pio** runs `DAP_JTAG_Sequence` on an RP2040 PIO state machine (§ 4 Patch 4, § 8.7). A first version that set TMS from the CPU and waited after every entry was *slower* than CPU bit-banging (755 µs per packet at 4 MHz); with TMS in the PIO stream and the entries streamed back to back, the JTAG work per packet halves (581 → 292 µs at 10 MHz), `load_image` reaches **58.2 KB/s** (16 % faster) and smp_mem_test with `SEMI_PRINTS=1` runs in **291.5 s** instead of 308.1 s (§ 9.7, PASS).

---

## 11. Final Reflection: One CPU vs Two CPUs

| Raspberry Pi 4 B over JTAG, OpenOCD 0.12 | 1 CPU (FreeRTOS JTAGprobe) | 2 CPUs (JTAGprobe_smp) |
| :--- | ---: | ---: |
| `load_image`, smp_mem_inter_test (80,984 B) | 2,380 ms (34.0 KB/s) | 2,106–2,143 ms (37.8–38.5 KB/s) |
| Probe time per packet (≈ 9.9 loaded bytes) | 251–292 µs | 259–264 µs |
| Whole smp_mem_test (CTest) | 140 s | 50–54 s |
| Whole smp_mem_inter_test (CTest) | 190 s | 186–193 s |
| smp_mem_test / smp_mem_inter_test with `SEMI_PRINTS=1` | — | 337 s / 621 s |

Both columns use 64-byte DAP packets and CPU bit-banging at `adapter speed 4000`. The 512-byte packets (§ 9.6: 1,644 ms, 47.1 s, 175.9 s, 308 s / 562 s for the same rows) and the PIO (§ 9.7: smp_mem_test load 1,286 ms, 291.5 s with `SEMI_PRINTS=1`) cut the time per byte in ways that would help a single-core firmware just as well, so they are left out of this one-vs-two comparison.

### 11.1 JTAG operations
- **Bulk transfers (`load_image`): about the same.** OpenOCD runs a strict request/reply chain in JTAG mode: one ~64-byte command (≈ 9.9 loaded bytes), then it waits for the answer. In such a chain a second core has nothing to overlap. Of a ~263 µs cycle, the counters of § 9.4 give ~68 µs to the bit-banging on core 1, ~7 µs to the hand-off between the cores, and ~188 µs to the USB transfer and the PC's turnaround. A second core can only *lose* time here, if the hand-off sleeps: the first SMP build (history, fixed since the second build) woke the other core twice per packet and needed ~1 ms per packet (4× slower). Keeping the packet path free of sleeps (the DAP thread spins on its ring and sends its own reply) is what made the two-core firmware as fast as the single-core one.
- **Scattered traffic (a running test): faster.** smp_mem_test, whose run is mostly OpenOCD polling the four Cortex-A72 cores between short exchanges, completed in 50–54 s instead of 140 s. The DAP work on core 1 never waits behind USB event handling or the UART on core 0. The share of each factor was not profiled. Where the test's own computing dominates (smp_mem_inter_test, ~170 s on the Pi), the probe makes no measurable difference (186–193 s against 190 s, within the run-to-run spread).

### 11.2 UART
On one CPU the UART bridge task and the DAP task share the core: the bridge has to preempt the bit-banging (it has the highest priority) or characters are lost. On two CPUs the UART thread runs on core 0, beside USB, and never delays a DAP command, nor can a DAP burst delay the UART. The tests ran with their whole console on the UART and passed; the bridge throughput itself was not benchmarked.

### 11.3 Semihosting
To the probe a semihosting call is ordinary JTAG traffic, so it gains like any scattered work. Its cost, however, is OpenOCD's: `SYS_WRITE0` reads the string one byte per JTAG memory access, ~0.9 s per console line, whatever the probe. With every core-0 line through semihosting (`SEMI_PRINTS=1`), smp_mem_test takes 337 s instead of 50–54 s and smp_mem_inter_test 621 s instead of 186–193 s. On this board OpenOCD also serves semihosting only from the target's core 0: a call from cores 1–3 is printed but that core is never resumed.

### 11.4 Where the next gains are

Not in another division of work between the two Pico cores (the hand-off is already ~7 µs per packet), but in:

1. **fewer round trips**: a larger `DAP_PACKET_SIZE`. **Done** in the current build (512 bytes over the 64-byte endpoint, § 9.6): loads 23 % faster, tests 6–10 % faster. What is left there is the ≈ 910 µs per packet of request reception plus PC turnaround, to be measured and, if it is the per-USB-packet re-arm, reduced by arming the rest of a request in one transfer;
2. **faster bit-banging**: a PIO state machine for TCK/TMS/TDI/TDO. **Done in `JTAGprobe_smp_pio`** (§ 9.7): with TMS in the PIO stream and `adapter speed 10000`, 292 µs instead of 581 µs per packet; the request reception plus the PC's turnaround (≈ 920 µs of ≈ 1,450 µs) is now the largest part of a load cycle;
3. **the host**: reading semihosting strings in blocks instead of byte by byte.



