/* APU on Core 1 - Parallel SPC700 emulation */

#ifndef APU_CORE1_H
#define APU_CORE1_H

#ifdef PICO_ON_DEVICE

#include <stdint.h>
#include <stdbool.h>

/* Enable Core 1 APU processing - set to 1 to enable */
#define APU_ON_CORE1 1

/* Shared state between cores */
extern volatile int32_t apu_target_cycles;
extern volatile int32_t apu_cycle_debt;  /* Core 0 accumulates, Core 1 applies */
extern volatile bool apu_core1_enabled;

/* Initialize Core 1 APU state */
void apu_core1_init(void);

/* Core 0: Set target cycle count for APU (non-blocking) */
void apu_core1_set_target_cycles(int32_t target);

/* Core 1: Run APU for a batch of cycles */
void apu_core1_run_batch(void);

/* Check if APU has caught up */
bool apu_core1_is_caught_up(void);

/* Replacement macros for APU_EXECUTE when using Core 1 */
#if APU_ON_CORE1
#define APU_EXECUTE_CORE1() apu_core1_set_target_cycles(CPU.Cycles)
#define APU_EXECUTE1_CORE1() apu_core1_set_target_cycles(CPU.Cycles)
#endif

#endif /* PICO_ON_DEVICE */

#endif /* APU_CORE1_H */
