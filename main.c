#include <linux/limits.h>
#include <stdio.h>

#define UTIL_IMPL
#include "util.h"


// reserved memory pages
#define ZERO_PAGE (0)
#define STACK (0x100)
#define MMIO (0xFF00)

// handler addresses
#define NMI (0xFFFA)
#define RES (0xFFFC)
#define IRQ (0xFFFE)

#define C (0x01) // carry
#define Z (0x02) // zero
#define I (0x04) // irq disable
#define D (0x08) // decimal (bcd arithmetic)
#define B (0x10) // break. 0 if pushed to stack by irq. 1 if pushed by instruction
#define O (0x20) // unused bit, always pushed to stack as 1
#define V (0x40) // overflow
#define N (0x80) // negative

#define ZMASK(b) ((u8)(b == 0) << 1)
#define NMASK(b) ((b) & N)

#define ADDR(lo, hi) ((u16)(lo) | ((u16)(hi) << 8))

#define CLRMASK(byte, mask) ((byte) & ~(mask)) // clear mask bits in byte

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
    u8 mem[0x10000];
} mem_t;

typedef struct {
    cpu_t *cpu;
    mem_t *mem;
    u8 oplo;
    u8 ophi;
} ctx_t;

typedef void (*isn_t)(ctx_t *ctx);

void emu_init(cpu_t *cpu, mem_t *mem) {
    cpu->pc = ADDR(mem->mem[RES], mem->mem[RES+1]);
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->p = 0;
    cpu->s = 0;
}

void print_registers(cpu_t *cpu, mem_t *mem) {
    util_log(UTIL_INFO, " PC  IRQ  SR AC XR YR SP");
    util_log(UTIL_INFO, "%04X %02X%02X %02X %02X %02X %02X %02X",
             cpu->pc, mem->mem[IRQ+1], mem->mem[IRQ], cpu->p,
             cpu->a, cpu->x, cpu->y, cpu->s);
}

void push_to_stack(cpu_t *cpu, mem_t *mem, u8 b) {
    if (cpu->s == 0)
        util_log(UTIL_WARN, "%s: stack pointer reached $00, wrapping around.", __func__);
    mem->mem[STACK + cpu->s--] = b;
}
u8 pull_from_stack(cpu_t *cpu, mem_t *mem) {
    if (cpu->s == 0xFF)
        util_log(UTIL_WARN, "%s: stack pointer reached $FF, wrapping around.", __func__);
    return mem->mem[STACK + cpu->s++];
}

#define ACC(ctx) ((ctx)->a)
#define ABS(ctx) ((ctx)->mem->mem[ADDR((ctx)->oplo, (ctx)->ophi)])
#define ABX(ctx) ((ctx)->mem->mem[(u16)(ADDR((ctx)->oplo, (ctx)->ophi)+(ctx)->x)])
#define ABY(ctx) ((ctx)->mem->mem[(u16)(ADDR((ctx)->oplo, (ctx)->ophi)+(ctx)->y)])
#define IMM(ctx) ((ctx)->oplo)
#define IMP(ctx)
#define IND(ctx)
#define XIN(ctx)
#define INY(ctx)
#define REL(ctx)
#define ZPG(ctx) ((ctx)->mem->mem[(ctx)->oplo])
#define ZPX(ctx) ((ctx)->mem->mem[(u16)((ctx)->oplo+(ctx)->x)])
#define ZPY(ctx) ((ctx)->mem->mem[(u16)((ctx)->oplo+(ctx)->y)])

void nop(ctx_t *ctx) { UNUSED(ctx); }

void oraimm(ctx_t *ctx) {
    ctx->cpu->a |= ctx->oplo;
}
void orazpg(ctx_t *ctx) {
    ctx->cpu->a |= ctx->mem->mem[ctx->oplo];
}
void orazpx(ctx_t *ctx) {
    ctx->cpu->a |= ctx->mem->mem[(u8)(ctx->oplo + ctx->cpu->x)];
}
void oraabs(ctx_t *ctx) {
    ctx->cpu->a |= ctx->mem->mem[ADDR(ctx->oplo, ctx->ophi)];
}
void oraabx(ctx_t *ctx) {
    ctx->cpu->a |= ctx->mem->mem[ADDR(ctx->oplo, ctx->ophi) + ctx->cpu->x];
}
void oraaby(ctx_t *ctx) {
    ctx->cpu->a |= ctx->mem->mem[ADDR(ctx->oplo, ctx->ophi) + ctx->cpu->y];
}
void oraxin(ctx_t *ctx) {
    u16 a = ADDR(ctx->mem->mem[ctx->oplo+ctx->cpu->x], ctx->mem->mem[(u8)(ctx->oplo+ctx->cpu->x+1)]);
    ctx->cpu->a |= ctx->mem->mem[a];
}
void orainy(ctx_t *ctx) {
    u16 a = ADDR(ctx->mem->mem[ctx->oplo], ctx->mem->mem[(u8)(ctx->oplo+1)]) + ctx->cpu->y;
    ctx->cpu->a |= ctx->mem->mem[a];
}

void andimm(ctx_t *ctx) {
    ctx->cpu->a &= ctx->oplo;
}
void andzpg(ctx_t *ctx) {
    ctx->cpu->a &= ctx->mem->mem[ctx->oplo];
}
void andzpx(ctx_t *ctx) {
    ctx->cpu->a &= ctx->mem->mem[(u8)(ctx->oplo + ctx->cpu->x)];
}
void andabs(ctx_t *ctx) {
    ctx->cpu->a &= ctx->mem->mem[ADDR(ctx->oplo, ctx->ophi)];
}
void andabx(ctx_t *ctx) {
    ctx->cpu->a &= ctx->mem->mem[ADDR(ctx->oplo, ctx->ophi) + ctx->cpu->x];
}
void andaby(ctx_t *ctx) {
    ctx->cpu->a &= ctx->mem->mem[ADDR(ctx->oplo, ctx->ophi) + ctx->cpu->y];
}
void andxin(ctx_t *ctx) {
    u16 a = ADDR(ctx->mem->mem[ctx->oplo+ctx->cpu->x], ctx->mem->mem[(u8)(ctx->oplo+ctx->cpu->x+1)]);
    ctx->cpu->a &= ctx->mem->mem[a];
}
void andiny(ctx_t *ctx) {
    u16 a = ADDR(ctx->mem->mem[ctx->oplo], ctx->mem->mem[(u8)(ctx->oplo+1)]) + ctx->cpu->y;
    ctx->cpu->a &= ctx->mem->mem[a];
}

void brkimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void aslzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void phpimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void aslacc(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void aslabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void bplrel(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void aslzpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void clcimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void aslabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void jsrabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void bitzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rolzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void plpimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rolacc(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void bitabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rolabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void bmirel(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rolzpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void secimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rolabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rtiimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void eorxin(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void eorzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void lsrzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void phaimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void eorimm(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void lsracc(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void jmpabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void eorabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void lsrabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void bvcrel(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void eoriny(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void eorzpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void lsrzpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cliimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void eoraby(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void eorabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void lsrabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rtsimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void adcxin(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void adczpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rorzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void plaimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void adcimm(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void roracc(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void jmpind(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void adcabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rorabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void bvsrel(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void adciny(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void adczpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rorzpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void seiimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void adcaby(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void adcabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void rorabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void staxin(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void styzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void stazpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void stxzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void deyimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void txaimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void styabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void staabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void stxabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void bccrel(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void stainy(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void styzpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void stazpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void stxzpy(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void tyaimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void staaby(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void txsimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void staabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldyimm(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldaxin(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldximm(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldyzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldazpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldxzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void tayimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldaimm(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void taximp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldyabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldaabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldxabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void bcsrel(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldainy(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldyzpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldazpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void pdxzpy(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void clvimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldaaby(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void tsximp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldyabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldaabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void ldxaby(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cpyimm(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cmpxin(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cpyzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cmpzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void deczpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void inyimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cmpimm(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void deximp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cpyabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cmpabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void decabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void bnerel(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cmpiny(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cmpzpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void deczpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cldimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cmpaby(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cmpabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void decabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cpximm(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void sbcxin(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cpxzpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void sbczpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void inczpg(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void inximp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void sbcimm(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void nopimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void cpxabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void sbcabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void incabs(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void beqrel(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void sbciny(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void sbczpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void inczpx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void sedimp(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void sbcaby(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void sbcabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }
void incabx(ctx_t *ctx) { UNUSED(ctx); UNIMPL; }

const isn_t isns[256] = {
/*           -0,     -1,     -2,     -3,     -4,     -5,     -6,     -7,     -8,     -9,     -A,     -B,     -C,     -D,     -E,     -F, */
/* 0- */ brkimp, oraxin,    nop,    nop,    nop, orazpg, aslzpg,    nop, phpimp, oraimm, aslacc,    nop,    nop, oraabs, aslabs,    nop,
/* 1- */ bplrel, orainy,    nop,    nop,    nop, orazpx, aslzpx,    nop, clcimp, oraaby,    nop,    nop,    nop, oraabx, aslabx,    nop,
/* 2- */ jsrabs, andxin,    nop,    nop, bitzpg, andzpg, rolzpg,    nop, plpimp, andimm, rolacc,    nop, bitabs, andabs, rolabs,    nop,
/* 3- */ bmirel, andiny,    nop,    nop,    nop, andzpx, rolzpx,    nop, secimp, andaby,    nop,    nop,    nop, andabx, rolabx,    nop,
/* 4- */ rtiimp, eorxin,    nop,    nop,    nop, eorzpg, lsrzpg,    nop, phaimp, eorimm, lsracc,    nop, jmpabs, eorabs, lsrabs,    nop,
/* 5- */ bvcrel, eoriny,    nop,    nop,    nop, eorzpx, lsrzpx,    nop, cliimp, eoraby,    nop,    nop,    nop, eorabx, lsrabx,    nop,
/* 6- */ rtsimp, adcxin,    nop,    nop,    nop, adczpg, rorzpg,    nop, plaimp, adcimm, roracc,    nop, jmpind, adcabs, rorabs,    nop,
/* 7- */ bvsrel, adciny,    nop,    nop,    nop, adczpx, rorzpx,    nop, seiimp, adcaby,    nop,    nop,    nop, adcabx, rorabx,    nop,
/* 8- */    nop, staxin,    nop,    nop, styzpg, stazpg, stxzpg,    nop, deyimp,    nop, txaimp,    nop, styabs, staabs, stxabs,    nop,
/* 9- */ bccrel, stainy,    nop,    nop, styzpx, stazpx, stxzpy,    nop, tyaimp, staaby, txsimp,    nop,    nop, staabx,    nop,    nop,
/* A- */ ldyimm, ldaxin, ldximm,    nop, ldyzpg, ldazpg, ldxzpg,    nop, tayimp, ldaimm, taximp,    nop, ldyabs, ldaabs, ldxabs,    nop,
/* B- */ bcsrel, ldainy,    nop,    nop, ldyzpx, ldazpx, pdxzpy,    nop, clvimp, ldaaby, tsximp,    nop, ldyabx, ldaabx, ldxaby,    nop,
/* C- */ cpyimm, cmpxin,    nop,    nop, cpyzpg, cmpzpg, deczpg,    nop, inyimp, cmpimm, deximp,    nop, cpyabs, cmpabs, decabs,    nop,
/* D- */ bnerel, cmpiny,    nop,    nop,    nop, cmpzpx, deczpx,    nop, cldimp, cmpaby,    nop,    nop,    nop, cmpabx, decabx,    nop,
/* E- */ cpximm, sbcxin,    nop,    nop, cpxzpg, sbczpg, inczpg,    nop, inximp, sbcimm, nopimp,    nop, cpxabs, sbcabs, incabs,    nop,
/* F- */ beqrel, sbciny,    nop,    nop,    nop, sbczpx, inczpx,    nop, sedimp, sbcaby,    nop,    nop,    nop, sbcabx, incabx,    nop,
};

int
main(int argc, char *argv[])
{
    printf("Hello, 6502!\n");

    return 0;
}
