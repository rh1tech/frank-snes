/*
 * Minimal stand-in for libretro-common's file_stream_transforms.h.
 *
 * snes9x routes all of its file I/O through these wrappers. On the RP2350
 * every user of them is disabled — savestates go through the port's own SD
 * code, and cheats, movies, screenshots and the config file are not built —
 * so the whole layer reduces to a type and a set of stubs. Providing them
 * here keeps the core source unmodified and avoids importing libretro-common
 * for functions that would never be called.
 */
#ifndef PORT_FILE_STREAM_TRANSFORMS_H
#define PORT_FILE_STREAM_TRANSFORMS_H

#include <stddef.h>
#include <stdint.h>

typedef struct RFILE RFILE;

#ifdef __cplusplus
extern "C" {
#endif

static inline RFILE *rfopen(const char *p, const char *m) { (void)p; (void)m; return 0; }
static inline int    rfclose(RFILE *f) { (void)f; return 0; }
static inline size_t rfread(void *b, size_t s, size_t n, RFILE *f) { (void)b;(void)s;(void)n;(void)f; return 0; }
static inline size_t rfwrite(const void *b, size_t s, size_t n, RFILE *f) { (void)b;(void)s;(void)n;(void)f; return 0; }
static inline int    rfseek(RFILE *f, long o, int w) { (void)f;(void)o;(void)w; return -1; }
static inline long   rftell(RFILE *f) { (void)f; return -1; }
static inline char  *rfgets(char *b, int n, RFILE *f) { (void)b;(void)n;(void)f; return 0; }
static inline int    rfgetc(RFILE *f) { (void)f; return -1; }
static inline int    rfeof(RFILE *f) { (void)f; return 1; }
static inline int    rfflush(RFILE *f) { (void)f; return 0; }
static inline int    rfprintf(RFILE *f, const char *fmt, ...) { (void)f;(void)fmt; return 0; }

#ifdef __cplusplus
}
#endif
#endif
