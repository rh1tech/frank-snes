/* APU on Core 1 - Parallel SPC700 emulation
 *
 * This module runs the SPC700 APU on Core 1 in parallel with the 65816 CPU on Core 0.
 * 
 * Architecture:
 * - Core 0 runs CPU and updates target cycle count atomically
 * - Core 1 runs APU to catch up whenever it has spare cycles
 * - No blocking synchronization needed for normal operation
 * - Port reads/writes use the existing atomic port buffers
 */

#ifdef PICO_ON_DEVICE

#include "apu_core1.h"
#include "apu.h"
#include "spc700.h"
#include "cpuexec.h"
#include "soundux.h"
#include "pico.h"
#include "hardware/sync.h"
#include <string.h>

/* Shared state between cores - aligned for atomic access */
volatile int32_t __attribute__((aligned(4))) apu_target_cycles = 0;
volatile int32_t __attribute__((aligned(4))) apu_cycle_debt = 0;
volatile bool apu_core1_enabled = false;

void apu_core1_init(void)
{
    apu_target_cycles = 0;
    apu_cycle_debt = 0;
    apu_core1_enabled = true;
    __dmb();
}

/* Core 0 calls this to update target - non-blocking */
void __not_in_flash_func(apu_core1_set_target_cycles)(int32_t target)
{
    apu_target_cycles = target;
}

/* Run APU until caught up - called from Core 1 render loop */
void __not_in_flash_func(apu_core1_run_batch)(void)
{
    if (!apu_core1_enabled) return;

    /* Do nothing unless Core 0 is actually driving this path.
     *
     * It is not. APU_EXECUTE()/APU_EXECUTE1() only expand to the Core 1
     * versions if APU_ON_CORE1 is already defined when spc700.h is parsed,
     * and spc700.h includes apu_core1.h INSIDE that test - so in every
     * translation unit the default, synchronous, Core 0 macros win.
     * apu_core1_set_target_cycles is not even present in the linked image,
     * and a sentinel written over apu_target_cycles survives indefinitely:
     * nothing ever writes it.
     *
     * That makes everything below dead code - except that it was not inert.
     * `APU.Cycles -= debt` is a non-atomic read-modify-write on state that
     * Core 0 is updating inside APUExecute(), executed thousands of times a
     * second from Core 1's loop. A stale write-back rewinds the APU's
     * emulated clock, and CPU<->APU handshakes are exactly what that breaks:
     * the 65816 sits at $83:BE74 waiting for a port value the SPC700 is
     * running too early or too late to produce, with the picture frozen and
     * every other counter healthy.
     *
     * Writing IAPU.APUExecuting from here is the same hazard in miniature. */
    if (apu_target_cycles == 0) return;

    /* Always keep SPC700 running — SLEEP/STOP are treated as NOPs */
    IAPU.APUExecuting = true;

    /* Apply cycle debt from Core 0 (HBlank adjustments).
     * Atomic exchange: read the accumulated debt and reset to 0.
     * This avoids a race where Core 0 writes APU.Cycles while Core 1
     * is also modifying it (lost update → timing drift → crash). */
    int32_t debt = __atomic_exchange_n(&apu_cycle_debt, 0, __ATOMIC_RELAXED);
    APU.Cycles -= debt;

    int32_t target = apu_target_cycles;

    while (APU.Cycles < target) {
        APUExecute();
    }
}

/* Check if APU has caught up to target */
bool __not_in_flash_func(apu_core1_is_caught_up)(void)
{
    return APU.Cycles >= apu_target_cycles;
}

#endif /* PICO_ON_DEVICE */
