/*
 * frank-snes — record a hard fault instead of freezing the picture
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * A Cortex-M that takes a fault inside a fault escalates to LOCKUP and parks
 * the PC at 0xEFFFFFFE with the core stopped. On this board that is invisible
 * from the outside in the worst possible way: core 1 keeps scanning the last
 * framebuffer out to HDMI, so the display holds a perfectly good still image
 * and the only symptom is that the game "froze". Measured on a four-minute
 * soak with the slave absent entirely: the picture stopped changing, held for
 * a minute, then went black. Reading the PC afterwards gave 0xEFFFFFFE.
 *
 * Two things are wrong with that, and this fixes both:
 *
 *   - it is unrecoverable without a human, and
 *   - it destroys the evidence, because by the time anyone looks the only
 *     thing left is "stopped".
 *
 * So the fault handler writes the stacked PC, the stacked LR and CFSR into
 * the watchdog scratch registers - which survive the reboot it then triggers -
 * and the next boot publishes them. A frozen picture becomes a reboot plus an
 * address that can be fed to addr2line.
 */
#include <stdint.h>
#include <stdio.h>

#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "pico/stdlib.h"

/* Scratch 0..3 are left alone; the slave's boot-failure counter uses 0 and
   this keeps the same convention on both chips. */
#define FAULT_SCRATCH_MAGIC_IDX 4
#define FAULT_SCRATCH_PC_IDX    5
#define FAULT_SCRATCH_LR_IDX    6
#define FAULT_SCRATCH_CFSR_IDX  7
#define FAULT_MAGIC             0xFA017EDDu

/* Published for telemetry, so the fault can be read without a debugger. */
volatile uint32_t frank_fault_pc;
volatile uint32_t frank_fault_lr;
volatile uint32_t frank_fault_cfsr;
volatile uint32_t frank_fault_count;

/* Called from the naked handler below with a pointer to the exception frame.
   The frame is r0,r1,r2,r3,r12,lr,pc,xpsr - so [5] is LR and [6] is PC. */
void __attribute__((used)) frank_fault_record(uint32_t *frame)
{
    uint32_t prev = (watchdog_hw->scratch[FAULT_SCRATCH_MAGIC_IDX] >> 8) ==
                    (FAULT_MAGIC >> 8)
                  ? (watchdog_hw->scratch[FAULT_SCRATCH_MAGIC_IDX] & 0xffu) : 0u;
    if (prev < 0xffu) prev++;

    watchdog_hw->scratch[FAULT_SCRATCH_PC_IDX]   = frame[6];
    watchdog_hw->scratch[FAULT_SCRATCH_LR_IDX]   = frame[5];
    watchdog_hw->scratch[FAULT_SCRATCH_CFSR_IDX] =
        *(volatile uint32_t *) 0xE000ED28u;      /* CFSR */
    watchdog_hw->scratch[FAULT_SCRATCH_MAGIC_IDX] =
        (FAULT_MAGIC & 0xffffff00u) | prev;

    /* Reboot rather than lock up. The alternative is a still picture and a
       dead machine, which is what this exists to stop. */
    watchdog_reboot(0, 0, 0);
    for (;;) tight_loop_contents();
}

/* Weak in the SDK's vector table; this replaces it.
 *
 * Naked, because the whole point is to read the frame the hardware pushed
 * before the compiler pushes anything of its own. Bit 2 of EXC_RETURN says
 * which stack it went on. */
void __attribute__((naked)) isr_hardfault(void)
{
    __asm volatile (
        "movs r1, #4              \n"
        "mov  r0, lr              \n"
        "tst  r0, r1              \n"
        "beq  1f                  \n"
        "mrs  r0, psp             \n"
        "b    2f                  \n"
        "1:                       \n"
        "mrs  r0, msp             \n"
        "2:                       \n"
        "ldr  r2, =frank_fault_record \n"
        "bx   r2                  \n"
    );
}

/* Call once, early, after stdio is up. Reports and clears any record left by
   the boot before this one. */
void frank_fault_report_previous(void)
{
    uint32_t magic = watchdog_hw->scratch[FAULT_SCRATCH_MAGIC_IDX];
    if ((magic >> 8) != (FAULT_MAGIC >> 8)) return;

    frank_fault_count = magic & 0xffu;
    frank_fault_pc    = watchdog_hw->scratch[FAULT_SCRATCH_PC_IDX];
    frank_fault_lr    = watchdog_hw->scratch[FAULT_SCRATCH_LR_IDX];
    frank_fault_cfsr  = watchdog_hw->scratch[FAULT_SCRATCH_CFSR_IDX];

    printf("[fault] previous boot died: pc=%08lx lr=%08lx cfsr=%08lx (fault #%lu)\n",
           (unsigned long) frank_fault_pc, (unsigned long) frank_fault_lr,
           (unsigned long) frank_fault_cfsr, (unsigned long) frank_fault_count);
}
