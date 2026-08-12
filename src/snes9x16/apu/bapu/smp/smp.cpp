#ifdef DEBUGGER
#include "../../../snes9x.h"
#include "../../../debug.h"
char tmp[1024];
#endif

#include "../snes/snes.hpp"

#define SMP_CPP
namespace SNES {

#ifdef DEBUGGER
#include "debugger/disassembler.cpp"
#endif

SMP smp;

#include "algorithms.cpp"
#include "core.cpp"
#include "iplrom.cpp"
#include "memory.cpp"
#include "timing.cpp"

void SMP::enter() {
  while(clock < 0) op_step();
}

void SMP::power() {
  Processor::clock = 0;

  timer0.target = 0;
  timer1.target = 0;
  timer2.target = 0;

  reset();
}

void SMP::reset() {
  for(unsigned n = 0x0000; n <= 0xffff; n++) apuram[n] = 0x00;

  opcode_number = 0;
  opcode_cycle = 0;

  regs.pc = 0xffc0;
  regs.sp = 0xef;
  regs.B.a = 0x00;
  regs.x = 0x00;
  regs.B.y = 0x00;
  regs.p = 0x02;

  //$00f1
  status.iplrom_enable = true;

  //$00f2
  status.dsp_addr = 0x00;

  //$00f8,$00f9
  status.ram00f8 = 0x00;
  status.ram00f9 = 0x00;

  //timers
  timer0.enable = timer1.enable = timer2.enable = false;
  timer0.stage1_ticks = timer1.stage1_ticks = timer2.stage1_ticks = 0;
  timer0.stage2_ticks = timer1.stage2_ticks = timer2.stage2_ticks = 0;
  timer0.stage3_ticks = timer1.stage3_ticks = timer2.stage3_ticks = 0;
}

SMP::SMP() {
#ifdef FRANK_SNES_CPU_CORE_S9X16
  /* SNES::smp is a global, so this constructor runs before main() - before
     psram_init(), and against the few tens of KB of SRAM heap the SDK has
     at that point. Allocating 64 KB here panics the board with "out of
     memory" before a single line of the port's own code has run.
     S9xInitAPU() allocates it instead, once there is somewhere to put it. */
  apuram = NULL;
#else
  apuram = new uint8[64 * 1024];
#endif
}

SMP::~SMP() {
#ifdef FRANK_SNES_CPU_CORE_S9X16
	/* allocated with the port allocator in S9xInitAPU, not with new */
#else
	delete[] apuram;
#endif
}

}
