#ifndef EMU_H
#define EMU_H

#define MEMSIZE (0x10000)

// non-maskable interrupt vector
#define NMI (0xFFFA)

// reset vector
#define RES (0xFFFC)

// hardware interrupt vector
#define IRQ (0xFFFE)

typedef unsigned char u8;
typedef char i8;
typedef unsigned short u16;
typedef short i16;

typedef struct {
    u16 pc; // program counter lo
    u8 a; // accumulator
    u8 x; // index register x
    u8 y; // index register y
    u8 p; // processor status
    u8 s; // stack pointer
} cpu_t;

typedef struct {
    u8 *mem; // size: 0x10000
    cpu_t cpu;
} emu_t;

void emu_init(emu_t *e, const u8 *prog, u16 size);
void emu_end(emu_t *e);
void emu_step(emu_t *e);
void emu_log_state(emu_t *e);

#endif // EMU_H
