/*
 * Compatibility shim for building snes9x 1.6x on the RP2350.
 *
 * The core expects a hosted C library and libretro's file_stream wrappers.
 * On the device there is no filesystem behind those calls — savestates,
 * cheats, movies and the config file are all disabled — so they are stubbed
 * here rather than dragging libretro-common in.
 */
#ifndef PORT_SHIM_H
#define PORT_SHIM_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STREAM              FILE *
#define OPEN_STREAM(f, m)   NULL
#define REOPEN_STREAM(f, m) NULL
#define FIND_STREAM(f)      0
#define REVERT_STREAM(f, o, s)
#define CLOSE_STREAM(s)
#define READ_STREAM(p, l, s)  0
#define WRITE_STREAM(p, l, s) 0
#define GETS_STREAM(p, l, s)  NULL
#define GETC_STREAM(s)        (-1)

#endif
