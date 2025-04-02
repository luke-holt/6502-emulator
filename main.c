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

#define B0 (0x01)
#define B1 (0x02)
#define B2 (0x04)
#define B3 (0x08)
#define B4 (0x10)
#define B5 (0x20)
#define B6 (0x40)
#define B7 (0x80)

#define C (B0) // carry
#define Z (B1) // zero
#define I (B2) // irq disable
#define D (B3) // decimal (bcd arithmetic)
#define B (B4) // break. 0 if pushed to stack by irq. 1 if pushed by instruction
#define O (B5) // unused bit, always pushed to stack as 1
#define V (B6) // overflow
#define N (B7) // negative

#define ZMASK(b) ((u8)((b) == 0) << 1)
#define NMASK(b) ((b) & N)
#define VMASK(b) ((b) & V)


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
    u8 *mem; // size: 0x10000
    cpu_t cpu;
    u8 oplo;
    u8 ophi;
} emu_t;

static inline u16 addr(u8 lo, u8 hi) {
    return lo | (hi << 8);
}
static inline u16 addrat(emu_t *e, u8 lo, u8 hi) {
    u16 a = addr(lo, hi);
    return addr(e->mem[a], e->mem[a+1]);
}

typedef void (*isn_t)(emu_t *e);

void emu_init(emu_t *e) {
    e->cpu.pc = addrat(e, RES);
    e->cpu.a = 0;
    e->cpu.x = 0;
    e->cpu.y = 0;
    e->cpu.p = 0;
    e->cpu.s = 0;
}

void print_registers(emu_t *e) {
    util_log(UTIL_INFO, " PC  IRQ  SR AC XR YR SP");
    util_log(UTIL_INFO, "%04X %04X %02X %02X %02X %02X %02X",
             e->cpu.pc, addrat(e, IRQ), e->cpu.p,
             e->cpu.a, e->cpu.x, e->cpu.y, e->cpu.s);
}

void push_to_stack(emu_t *e, u8 b) {
    if (e->cpu.s == 0)
        util_log(UTIL_WARN, "%s: stack pointer underflow", __func__);
    e->mem[STACK + e->cpu.s--] = b;
}
u8 pull_from_stack(emu_t *e) {
    if (e->cpu.s == 0xFF)
        util_log(UTIL_WARN, "%s: stack pointer overflow", __func__);
    return e->mem[STACK + e->cpu.s++];
}

inline u8 acc(emu_t *e) { return e->cpu.a; }
inline u8 ast(emu_t *e) { return e->mem[addr(e->oplo, e->ophi)]; }
inline u8 abx(emu_t *e) { return e->mem[addr(e->oplo, e->ophi)+e->cpu.x]; }
inline u8 aby(emu_t *e) { return e->mem[addr(e->oplo, e->ophi)+e->cpu.y]; }
inline u8 imm(emu_t *e) { return e->oplo; }
// inline u8 imp(emu_t *e) { UNUSED(e); UNIMPL; return 0;}
inline u8 ind(emu_t *e) { return e->mem[addrat(e, e->oplo, e->ophi)]; }
inline u8 xin(emu_t *e) { return e->mem[addrat(e, e->oplo + e->cpu.x, 0)]; }
inline u8 yin(emu_t *e) { return e->mem[addrat(e, e->oplo, 0) + e->cpu.y]; }
// inline u8 rel(emu_t *e) { UNUSED(e); UNIMPL; return 0;}
inline u8 zpg(emu_t *e) { return e->mem[e->oplo]; }
inline u8 zpx(emu_t *e) { return e->mem[addr(e->oplo + e->cpu.x, 0)]; }
inline u8 zpy(emu_t *e) { return e->mem[addr(e->oplo + e->cpu.y, 0)]; }

inline u8 *racc(emu_t *e) { return &e->cpu.a; }
inline u8 *rast(emu_t *e) { return &e->mem[addr(e->oplo, e->ophi)]; }
inline u8 *rabx(emu_t *e) { return &e->mem[addr(e->oplo, e->ophi)+e->cpu.x]; }
inline u8 *raby(emu_t *e) { return &e->mem[addr(e->oplo, e->ophi)+e->cpu.y]; }
// inline u8 *rimm(emu_t *e) { return &e->oplo; }
// inline u8 *rimp(emu_t *e) { UNUSED(e); UNIMPL; return 0;}
inline u8 *rind(emu_t *e) { return &e->mem[addrat(e, e->oplo, e->ophi)]; }
inline u8 *rxin(emu_t *e) { return &e->mem[addrat(e, e->oplo + e->cpu.x, 0)]; }
inline u8 *ryin(emu_t *e) { return &e->mem[addrat(e, e->oplo, 0) + e->cpu.y]; }
// inline u8 *rrel(emu_t *e) { UNUSED(e); UNIMPL; return 0;}
inline u8 *rzpg(emu_t *e) { return &e->mem[e->oplo]; }
inline u8 *rzpx(emu_t *e) { return &e->mem[addr(e->oplo + e->cpu.x, 0)]; }
inline u8 *rzpy(emu_t *e) { return &e->mem[addr(e->oplo + e->cpu.y, 0)]; }



// add memory to accumulator with carry
inline void adc(emu_t *e, u8 m) {
    u16 s = e->cpu.a + m + ((e->cpu.p & C) == C);
    u8 c = ((s >> 8) & C);
    e->cpu.p &= ~(Z|N|C|V);
    if (((e->cpu.a & B7) == (m & B7)) && ((s & B7) != (m & B7)))
        e->cpu.p |= V;
    e->cpu.a = (u8)s;
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a) | c;
}
void adcimm(emu_t *e) { adc(e, imm(e)); }
void adczpg(emu_t *e) { adc(e, zpg(e)); }
void adczpx(emu_t *e) { adc(e, zpx(e)); }
void adcabs(emu_t *e) { adc(e, ast(e)); }
void adcabx(emu_t *e) { adc(e, abx(e)); }
void adcaby(emu_t *e) { adc(e, aby(e)); }
void adcxin(emu_t *e) { adc(e, xin(e)); }
void adcyin(emu_t *e) { adc(e, yin(e)); }

// and memory with accumulator
inline void and(emu_t *e, u8 m) {
    e->cpu.a &= m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}
void andimm(emu_t *e) { and(e, imm(e)); }
void andzpg(emu_t *e) { and(e, zpg(e)); }
void andzpx(emu_t *e) { and(e, zpx(e)); }
void andabs(emu_t *e) { and(e, ast(e)); }
void andabx(emu_t *e) { and(e, abx(e)); }
void andaby(emu_t *e) { and(e, aby(e)); }
void andxin(emu_t *e) { and(e, xin(e)); }
void andyin(emu_t *e) { and(e, yin(e)); }

// shift left one bit (memory or accumulator)
inline void asl(emu_t *e, u8 *m) {
    u8 s = *m << 1;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= ZMASK(s) | NMASK(s) | ((*m >> 7) & C);
    *m = s;
}
void aslacc(emu_t *e) { asl(e, racc(e)); }
void aslzpg(emu_t *e) { asl(e, rzpg(e)); }
void aslzpx(emu_t *e) { asl(e, rzpx(e)); }
void aslabs(emu_t *e) { asl(e, rast(e)); }
void aslabx(emu_t *e) { asl(e, rabx(e)); }

// branch on carry clear
void bccrel(emu_t *e) { UNUSED(e); UNIMPL; }

// branch on carry set
void bcsrel(emu_t *e) { UNUSED(e); UNIMPL; }

// branch on result zero
void beqrel(emu_t *e) { UNUSED(e); UNIMPL; }

// test bits in memory with accumulator
inline void bit(emu_t *e, u8 m) {
    e->cpu.p &= ~(Z|N|V);
    e->cpu.p |= ZMASK(e->cpu.a & m) | NMASK(m) | VMASK(m);
}
void bitzpg(emu_t *e) { bit(e, zpg(e)); }
void bitabs(emu_t *e) { bit(e, ast(e)); }

// branch on result minus
inline void bmi(emu_t *e) { UNUSED(e); UNIMPL; }
void bmirel(emu_t *e) { UNUSED(e); UNIMPL; }

// branch on result not zero
inline void bnerel(emu_t *e) { UNUSED(e); UNIMPL; }

// branch on result plus
inline void bplrel(emu_t *e) { UNUSED(e); UNIMPL; }

// force break
inline void brk(emu_t *e) { UNUSED(e); UNIMPL; }
void brkimp(emu_t *e) { UNUSED(e); UNIMPL; }

// branch on overflow clear
inline void bvc(emu_t *e) { UNUSED(e); UNIMPL; }
void bvcrel(emu_t *e) { UNUSED(e); UNIMPL; }

// branch on overflow set
inline void bvs(emu_t *e) { UNUSED(e); UNIMPL; }
void bvsrel(emu_t *e) { UNUSED(e); UNIMPL; }

// clear carry flag
inline void clc(emu_t *e) { e->cpu.p &= ~C; }
void clcimp(emu_t *e) { clc(e); }

// clear decimal mode
inline void cld(emu_t *e) { e->cpu.p &= ~D; }
void cldimp(emu_t *e) { cld(e); }

// clear interrupt disable bit
inline void cli(emu_t *e) { e->cpu.p &= ~I; }
void cliimp(emu_t *e) { cli(e); }

// clear overflow flag
inline void clv(emu_t *e) { e->cpu.p &= ~V; }
void clvimp(emu_t *e) { clv(e); }

// compare memory with accumulator
inline void cmp(emu_t *e, u8 m) {
    u8 r = e->cpu.a - m;
    u8 c = (e->cpu.a >= m) & 1;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= ZMASK(r) | NMASK(r) | c;
}
void cmpimm(emu_t *e) { cmp(e, imm(e)); }
void cmpzpg(emu_t *e) { cmp(e, zpg(e)); }
void cmpzpx(emu_t *e) { cmp(e, zpx(e)); }
void cmpabs(emu_t *e) { cmp(e, abs(e)); }
void cmpabx(emu_t *e) { cmp(e, abx(e)); }
void cmpaby(emu_t *e) { cmp(e, aby(e)); }
void cmpxin(emu_t *e) { cmp(e, xin(e)); }
void cmpyin(emu_t *e) { cmp(e, yin(e)); }

// compare memory and index x
inline void cpx(emu_t *e, u8 m) {
    u8 r = e->cpu.x - m;
    u8 c = (e->cpu.x >= m) & 1;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= ZMASK(r) | NMASK(r) | c;
}
void cpximm(emu_t *e) { cpx(e, imm(e)); }
void cpxzpg(emu_t *e) { cpx(e, zpg(e)); }
void cpxabs(emu_t *e) { cpx(e, ast(e)); }

// compare memory and index y
inline void cpy(emu_t *e, u8 m) {
    u8 r = e->cpu.y - m;
    u8 c = (e->cpu.y >= m) & 1;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= ZMASK(r) | NMASK(r) | c;
}
void cpyimm(emu_t *e) { cpy(e, imm(e)); }
void cpyzpg(emu_t *e) { cpy(e, zpg(e)); }
void cpyabs(emu_t *e) { cpy(e, ast(e)); }

// decrement memory by one
void dec(emu_t *e, u8 *m) {
    (*m)--;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(*m) | NMASK(*m);
}
void deczpg(emu_t *e) { dec(e, rzpg(e)); }
void deczpx(emu_t *e) { dec(e, rzpx(e)); }
void decabs(emu_t *e) { dec(e, rast(e)); }
void decabx(emu_t *e) { dec(e, rabx(e)); }

// decrement index x by one
void dex(emu_t *e) {
    e->cpu.x--;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.x) | NMASK(e->cpu.x);
}
void deximp(emu_t *e) { dex(e); }

// decrement index y by one
void dey(emu_t *e) {
    e->cpu.y--;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.y) | NMASK(e->cpu.y);
}
void deyimp(emu_t *e) { dey(e); }

// exclusize-or memory with accumulator
void eor(emu_t *e, u8 m) {
    e->cpu.a ^= m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}
void eorimm(emu_t *e) { eor(e, imm(e)); }
void eorzpg(emu_t *e) { eor(e, zpg(e)); }
void eorzpx(emu_t *e) { eor(e, zpx(e)); }
void eorabs(emu_t *e) { eor(e, ast(e)); }
void eorabx(emu_t *e) { eor(e, abx(e)); }
void eoraby(emu_t *e) { eor(e, aby(e)); }
void eorxin(emu_t *e) { eor(e, xin(e)); }
void eoryin(emu_t *e) { eor(e, yin(e)); }

// increment memory by one
void inc(emu_t *e, u8 *m) {
    (*m)++;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(*m) | NMASK(*m);
}
void inczpg(emu_t *e) { inc(e, rzpg(e)); }
void inczpx(emu_t *e) { inc(e, rzpx(e)); }
void incabs(emu_t *e) { inc(e, rast(e)); }
void incabx(emu_t *e) { inc(e, rabx(e)); }

// increment x by one
void inx(emu_t *e) {
    e->cpu.x++;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.x) | NMASK(e->cpu.x);
}
void inximp(emu_t *e) { inx(e); }

// increment y by one
void iny(emu_t *e) {
    e->cpu.y++;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.y) | NMASK(e->cpu.y);
}
void inyimp(emu_t *e) { iny(e); }

// jump to new location
void jmp(emu_t *e) { UNUSED(e); UNIMPL; }
void jmpabs(emu_t *e) { UNUSED(e); UNIMPL; }
void jmpind(emu_t *e) { UNUSED(e); UNIMPL; }

// jump to new location saving return address
void jsr(emu_t *e) { UNUSED(e); UNIMPL; }
void jsrabs(emu_t *e) { UNUSED(e); UNIMPL; }

// load accumulator with memory
void lda(emu_t *e, u8 m) {
    e->cpu.a = m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}
void ldaimm(emu_t *e) { lda(e, imm(e)); }
void ldazpg(emu_t *e) { lda(e, zpg(e)); }
void ldazpx(emu_t *e) { lda(e, zpx(e)); }
void ldaabs(emu_t *e) { lda(e, ast(e)); }
void ldaabx(emu_t *e) { lda(e, abx(e)); }
void ldaaby(emu_t *e) { lda(e, aby(e)); }
void ldaxin(emu_t *e) { lda(e, xin(e)); }
void ldayin(emu_t *e) { lda(e, yin(e)); }

// load index x with memory
void ldx(emu_t *e, u8 m) {
    e->cpu.x = m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.x) | NMASK(e->cpu.x);
}
void ldximm(emu_t *e) { ldx(e, imm(e)); }
void ldxzpg(emu_t *e) { ldx(e, zpg(e)); }
void ldxzpy(emu_t *e) { ldx(e, zpy(e)); }
void ldxabs(emu_t *e) { ldx(e, ast(e)); }
void ldxaby(emu_t *e) { ldx(e, aby(e)); }

// load index y with memory
void ldy(emu_t *e, u8 m) {
    e->cpu.y = m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.y) | NMASK(e->cpu.y);
}
void ldyimm(emu_t *e) { ldy(e, imm(e)); }
void ldyzpg(emu_t *e) { ldy(e, zpg(e)); }
void ldyzpx(emu_t *e) { ldy(e, zpx(e)); }
void ldyabs(emu_t *e) { ldy(e, ast(e)); }
void ldyabx(emu_t *e) { ldy(e, abx(e)); }

void lsr(emu_t *e, u8 *m) {
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= *m & C;
    *m >>= 1;
    e->cpu.p |= ZMASK(*m);
}
void lsrzpg(emu_t *e) { lsr(e, rzpg(e)); }
void lsracc(emu_t *e) { lsr(e, racc(e)); }
void lsrabs(emu_t *e) { lsr(e, rast(e)); }
void lsrzpx(emu_t *e) { lsr(e, rzpx(e)); }
void lsrabx(emu_t *e) { lsr(e, rabx(e)); }

// no operation
void nop(emu_t *e) { UNUSED(e); }
void nopimp(emu_t *e) { nop(e); }

// or memory with accumulator
inline void ora(emu_t *e, u8 m) {
    e->cpu.a |= m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}
void oraimm(emu_t *e) { ora(e, imm(e)); }
void orazpg(emu_t *e) { ora(e, zpg(e)); }
void orazpx(emu_t *e) { ora(e, zpx(e)); }
void oraabs(emu_t *e) { ora(e, ast(e)); }
void oraabx(emu_t *e) { ora(e, abx(e)); }
void oraaby(emu_t *e) { ora(e, aby(e)); }
void oraxin(emu_t *e) { ora(e, xin(e)); }
void orayin(emu_t *e) { ora(e, yin(e)); }

// push accumulator on stack
void phaimp(emu_t *e) {
    push_to_stack(e, e->cpu.a);
}

// push processor status on stack
void phpimp(emu_t *e) {
    push_to_stack(e, e->cpu.p | B | O);
}

// pull accumulator from stack
void plaimp(emu_t *e) {
    e->cpu.a = pull_from_stack(e);
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}

// pull processor status from stack
void plpimp(emu_t *e) {
    e->cpu.p = pull_from_stack(e);
    e->cpu.p &= ~(B|O);
}

// rotate one bit left (memory or accumulator)
void rol(emu_t *e, u8 *m) {
    u8 c = e->cpu.p & C;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= ((*m & B7) >> 7) & C;
    *m = (*m << 1) | c;
    e->cpu.p |= ZMASK(*m) | NMASK(*m);
}
void rolacc(emu_t *e) { rol(e, racc(e)); }
void rolzpg(emu_t *e) { rol(e, rzpg(e)); }
void rolzpx(emu_t *e) { rol(e, rzpx(e)); }
void rolabs(emu_t *e) { rol(e, rast(e)); }
void rolabx(emu_t *e) { rol(e, rabx(e)); }

// rotate one bit right (memory or accumulator)
void ror(emu_t *e, u8 *m) {
    u8 c = e->cpu.p & C;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= *m & C;
    *m = (*m >> 1) | (c << 7);
    e->cpu.p |= ZMASK(*m) | NMASK(*m);
}
void roracc(emu_t *e) { ror(e, racc(e)); }
void rorzpg(emu_t *e) { ror(e, rzpg(e)); }
void rorzpx(emu_t *e) { ror(e, rzpx(e)); }
void rorabs(emu_t *e) { ror(e, rast(e)); }
void rorabx(emu_t *e) { ror(e, rabx(e)); }

// return from interrupt
void rti(emu_t *e) { UNUSED(e); UNIMPL; }
void rtiimp(emu_t *e) { UNUSED(e); UNIMPL; }

// return from subroutine
void rts(emu_t *e) { UNUSED(e); UNIMPL; }
void rtsimp(emu_t *e) { UNUSED(e); UNIMPL; }

// subtract memory from accumulator with borrow
inline void sbc(emu_t *e, u8 m) {
    i16 s = e->cpu.a - m - ((e->cpu.p & C) != C);
    u8 c = ((s >> 8) & C);
    e->cpu.p &= ~(Z|N|C|V);
    if (((e->cpu.a & B7) == (m & B7)) && ((s & B7) != (m & B7)))
        e->cpu.p |= V;
    e->cpu.a = (u8)(i8)s;
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a) | c;
}
void sbcimm(emu_t *e) { sbc(e, imm(e)); }
void sbczpg(emu_t *e) { sbc(e, zpg(e)); }
void sbczpx(emu_t *e) { sbc(e, zpx(e)); }
void sbcabs(emu_t *e) { sbc(e, ast(e)); }
void sbcabx(emu_t *e) { sbc(e, abx(e)); }
void sbcaby(emu_t *e) { sbc(e, aby(e)); }
void sbcxin(emu_t *e) { sbc(e, xin(e)); }
void sbcyin(emu_t *e) { sbc(e, yin(e)); }

// set carry flag
void secimp(emu_t *e) {
    e->cpu.p |= C;
}

// set decimal flag
void sedimp(emu_t *e) {
    e->cpu.p |= D;
}

// set interrupt disable status
void seiimp(emu_t *e) {
    e->cpu.p |= I;
}

// store accumulator in memory
void sta(emu_t *e, u8 *m) {
    *m = e->cpu.a;
}
void stazpg(emu_t *e) { sta(e, rzpg(e)); }
void stazpx(emu_t *e) { sta(e, rzpx(e)); }
void staabs(emu_t *e) { sta(e, rast(e)); }
void staabx(emu_t *e) { sta(e, rabx(e)); }
void staaby(emu_t *e) { sta(e, raby(e)); }
void staxin(emu_t *e) { sta(e, rxin(e)); }
void stayin(emu_t *e) { sta(e, ryin(e)); }

// store index x in memory
void stx(emu_t *e, u8 *m) {
    *m = e->cpu.x;
}
void stxzpg(emu_t *e) { stx(e, rzpg(e)); }
void stxzpy(emu_t *e) { stx(e, rzpy(e)); }
void stxabs(emu_t *e) { stx(e, rast(e)); }

// store index y in memory
void sty(emu_t *e, u8 *m) {
    *m = e->cpu.y;
}
void styzpg(emu_t *e) { sty(e, rzpg(e)); }
void styzpx(emu_t *e) { sty(e, rzpx(e)); }
void styabs(emu_t *e) { sty(e, rast(e)); }

// transfer accumulator to index x
void taximp(emu_t *e) {
    e->cpu.x = e->cpu.a;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.x) | NMASK(e->cpu.x);
}

// transfer accumulator to index y
void tayimp(emu_t *e) {
    e->cpu.y = e->cpu.a;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.y) | NMASK(e->cpu.y);
}

// transfer stack pointer to index x
void tsximp(emu_t *e) {
    e->cpu.x = e->cpu.s;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.x) | NMASK(e->cpu.x);
}

// transfer index x to accumulator
void txaimp(emu_t *e) {
    e->cpu.a = e->cpu.x;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}

// transfer index x to stack register
void txsimp(emu_t *e) {
    e->cpu.s = e->cpu.x;
}

// transfer index y to accumulator
void tyaimp(emu_t *e) {
    e->cpu.a = e->cpu.y;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}

const isn_t isns[256] = {
/*           -0,     -1,     -2,     -3,     -4,     -5,     -6,     -7,     -8,     -9,     -A,     -B,     -C,     -D,     -E,     -F, */
/* 0- */ brkimp, oraxin,    nop,    nop,    nop, orazpg, aslzpg,    nop, phpimp, oraimm, aslacc,    nop,    nop, oraabs, aslabs,    nop,
/* 1- */ bplrel, orayin,    nop,    nop,    nop, orazpx, aslzpx,    nop, clcimp, oraaby,    nop,    nop,    nop, oraabx, aslabx,    nop,
/* 2- */ jsrabs, andxin,    nop,    nop, bitzpg, andzpg, rolzpg,    nop, plpimp, andimm, rolacc,    nop, bitabs, andabs, rolabs,    nop,
/* 3- */ bmirel, andyin,    nop,    nop,    nop, andzpx, rolzpx,    nop, secimp, andaby,    nop,    nop,    nop, andabx, rolabx,    nop,
/* 4- */ rtiimp, eorxin,    nop,    nop,    nop, eorzpg, lsrzpg,    nop, phaimp, eorimm, lsracc,    nop, jmpabs, eorabs, lsrabs,    nop,
/* 5- */ bvcrel, eoryin,    nop,    nop,    nop, eorzpx, lsrzpx,    nop, cliimp, eoraby,    nop,    nop,    nop, eorabx, lsrabx,    nop,
/* 6- */ rtsimp, adcxin,    nop,    nop,    nop, adczpg, rorzpg,    nop, plaimp, adcimm, roracc,    nop, jmpind, adcabs, rorabs,    nop,
/* 7- */ bvsrel, adcyin,    nop,    nop,    nop, adczpx, rorzpx,    nop, seiimp, adcaby,    nop,    nop,    nop, adcabx, rorabx,    nop,
/* 8- */    nop, staxin,    nop,    nop, styzpg, stazpg, stxzpg,    nop, deyimp,    nop, txaimp,    nop, styabs, staabs, stxabs,    nop,
/* 9- */ bccrel, stayin,    nop,    nop, styzpx, stazpx, stxzpy,    nop, tyaimp, staaby, txsimp,    nop,    nop, staabx,    nop,    nop,
/* A- */ ldyimm, ldaxin, ldximm,    nop, ldyzpg, ldazpg, ldxzpg,    nop, tayimp, ldaimm, taximp,    nop, ldyabs, ldaabs, ldxabs,    nop,
/* B- */ bcsrel, ldayin,    nop,    nop, ldyzpx, ldazpx, ldxzpy,    nop, clvimp, ldaaby, tsximp,    nop, ldyabx, ldaabx, ldxaby,    nop,
/* C- */ cpyimm, cmpxin,    nop,    nop, cpyzpg, cmpzpg, deczpg,    nop, inyimp, cmpimm, deximp,    nop, cpyabs, cmpabs, decabs,    nop,
/* D- */ bnerel, cmpyin,    nop,    nop,    nop, cmpzpx, deczpx,    nop, cldimp, cmpaby,    nop,    nop,    nop, cmpabx, decabx,    nop,
/* E- */ cpximm, sbcxin,    nop,    nop, cpxzpg, sbczpg, inczpg,    nop, inximp, sbcimm, nopimp,    nop, cpxabs, sbcabs, incabs,    nop,
/* F- */ beqrel, sbcyin,    nop,    nop,    nop, sbczpx, inczpx,    nop, sedimp, sbcaby,    nop,    nop,    nop, sbcabx, incabx,    nop,
};

int
main(int argc, char *argv[])
{
    printf("Hello, 6502!\n");

    return 0;
}
