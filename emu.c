#include <string.h>

#include "emu.h"

#include "asm.h"
#include "insn.h" 
#include "util.h"

#define STACK (0x100)

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

static inline u16 addr(u8 lo, u8 hi) {
    return (u16)lo | ((u16)hi << 8);
}
static inline u16 addrat(emu_t *e, u16 a) {
    return addr(e->mem[a], e->mem[a+1]);
}

// push to stack
static void push(emu_t *e, u8 b) {
    if (e->cpu.s == 0) {
        ulog(UWARN, "%s: stack pointer underflow", __func__);
        emu_log_state(e);
    }
    e->mem[STACK + e->cpu.s--] = b;
}
// pull from stack
static u8 pull(emu_t *e) {
    if (e->cpu.s == 0xFF) {
        ulog(UWARN, "%s: stack pointer overflow", __func__);
        emu_log_state(e);
    }
    return e->mem[STACK + e->cpu.s++];
}

// get next instruction byte and inc pc
static inline u8 pcnext(emu_t *e) { return e->mem[e->cpu.pc++]; }

// accumulator
static inline u8 acc(emu_t *e) { return e->cpu.a; }
static inline u8 *racc(emu_t *e) { return &e->cpu.a; }

// absolute
static inline u8 ast(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    return e->mem[addr(lo, hi)];
}
static inline u8 *rast(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    return &e->mem[addr(lo, hi)];
}

// absolute, x-indexed
static inline u8 abx(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    return e->mem[addr(lo, hi)+e->cpu.x];
}
static inline u8 *rabx(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    return &e->mem[addr(lo, hi)+e->cpu.x];
}

// absolute, y-indexed
static inline u8 aby(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    return e->mem[addr(lo, hi)+e->cpu.y];
}
static inline u8 *raby(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    return &e->mem[addr(lo, hi)+e->cpu.y];
}

// immediate
static inline u8 imm(emu_t *e) { return pcnext(e); }

// indirect
static inline u8 ind(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    return e->mem[addrat(e, addr(lo, hi))];
}
static inline u8 *rind(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    return &e->mem[addrat(e, addr(lo, hi))];
}

// x-indexed, indirect
static inline u8 xin(emu_t *e) {
    u8 lo = pcnext(e);
    return e->mem[addrat(e, addr(lo + e->cpu.x, 0))];
}
static inline u8 *rxin(emu_t *e) {
    u8 lo = pcnext(e);
    return &e->mem[addrat(e, addr(lo + e->cpu.x, 0))];
}

// indirect, y-indexed
static inline u8 yin(emu_t *e) {
    u8 lo = pcnext(e);
    return e->mem[addrat(e, addr(lo, 0)) + e->cpu.y];
}
static inline u8 *ryin(emu_t *e) {
    u8 lo = pcnext(e);
    return &e->mem[addrat(e, addr(lo, 0)) + e->cpu.y];
}

// zeropage
static inline u8 zpg(emu_t *e) {
    return e->mem[pcnext(e)];
}
static inline u8 *rzpg(emu_t *e) {
    return &e->mem[pcnext(e)];
}

// zeropage, x-indexed
static inline u8 zpx(emu_t *e) {
    u8 lo = pcnext(e);
    return e->mem[addr(lo + e->cpu.x, 0)];
}
static inline u8 *rzpx(emu_t *e) {
    u8 lo = pcnext(e);
    return &e->mem[addr(lo + e->cpu.x, 0)];
}

// zeropage, y-indexed
static inline u8 zpy(emu_t *e) {
    u8 lo = pcnext(e);
    return e->mem[addr(lo + e->cpu.y, 0)];
}
static inline u8 *rzpy(emu_t *e) {
    u8 lo = pcnext(e);
    return &e->mem[addr(lo + e->cpu.y, 0)];
}


// add memory to accumulator with carry
static inline void adc(emu_t *e, u8 m) {
    u16 s = e->cpu.a + m + ((e->cpu.p & C) == C);
    u8 c = ((s >> 8) & C);
    e->cpu.p &= ~(Z|N|C|V);
    if (((e->cpu.a & B7) == (m & B7)) && ((s & B7) != (m & B7)))
        e->cpu.p |= V;
    e->cpu.a = (u8)s;
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a) | c;
}
static void adcimm(emu_t *e) { adc(e, imm(e)); }
static void adczpg(emu_t *e) { adc(e, zpg(e)); }
static void adczpx(emu_t *e) { adc(e, zpx(e)); }
static void adcabs(emu_t *e) { adc(e, ast(e)); }
static void adcabx(emu_t *e) { adc(e, abx(e)); }
static void adcaby(emu_t *e) { adc(e, aby(e)); }
static void adcxin(emu_t *e) { adc(e, xin(e)); }
static void adcyin(emu_t *e) { adc(e, yin(e)); }

// and memory with accumulator
static inline void and(emu_t *e, u8 m) {
    e->cpu.a &= m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}
static void andimm(emu_t *e) { and(e, imm(e)); }
static void andzpg(emu_t *e) { and(e, zpg(e)); }
static void andzpx(emu_t *e) { and(e, zpx(e)); }
static void andabs(emu_t *e) { and(e, ast(e)); }
static void andabx(emu_t *e) { and(e, abx(e)); }
static void andaby(emu_t *e) { and(e, aby(e)); }
static void andxin(emu_t *e) { and(e, xin(e)); }
static void andyin(emu_t *e) { and(e, yin(e)); }

// shift left one bit (memory or accumulator)
static inline void asl(emu_t *e, u8 *m) {
    u8 s = *m << 1;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= ZMASK(s) | NMASK(s) | ((*m >> 7) & C);
    *m = s;
}
static void aslacc(emu_t *e) { asl(e, racc(e)); }
static void aslzpg(emu_t *e) { asl(e, rzpg(e)); }
static void aslzpx(emu_t *e) { asl(e, rzpx(e)); }
static void aslabs(emu_t *e) { asl(e, rast(e)); }
static void aslabx(emu_t *e) { asl(e, rabx(e)); }

// branch on carry clear
static void bccrel(emu_t *e) {
    u8 o = pcnext(e);
    if ((e->cpu.p & C) != C) e->cpu.pc += (i8)o;
}

// branch on carry set
static void bcsrel(emu_t *e) {
    u8 o = pcnext(e);
    if ((e->cpu.p & C) == C) e->cpu.pc += (i8)o;
}

// branch on result zero
static void beqrel(emu_t *e) {
    u8 o = pcnext(e);
    if ((e->cpu.p & Z) == Z) e->cpu.pc += (i8)o;
}

// test bits in memory with accumulator
static inline void bit(emu_t *e, u8 m) {
    e->cpu.p &= ~(Z|N|V);
    e->cpu.p |= ZMASK(e->cpu.a & m) | NMASK(m) | VMASK(m);
}
static void bitzpg(emu_t *e) { bit(e, zpg(e)); }
static void bitabs(emu_t *e) { bit(e, ast(e)); }

// branch on result minus
static void bmirel(emu_t *e) {
    u8 o = pcnext(e);
    if ((e->cpu.p & N) == N) e->cpu.pc += (i8)o;
}

// branch on result not zero
static inline void bnerel(emu_t *e) {
    u8 o = pcnext(e);
    if ((e->cpu.p & Z) != Z) e->cpu.pc += (i8)o;
}

// branch on result plus
static inline void bplrel(emu_t *e) {
    u8 o = pcnext(e);
    if ((e->cpu.p & N) != N) e->cpu.pc += (i8)o;
}

// force break
static void brkimp(emu_t *e) {
    u16 a = e->cpu.pc + 2;
    push(e, (a >> 8) & 0xFF);
    push(e, a & 0xFF);
    push(e, e->cpu.p | B | O);
    e->cpu.pc = NMI;
}

// branch on overflow clear
static void bvcrel(emu_t *e) {
    u8 o = pcnext(e);
    if ((e->cpu.p & V) != V) e->cpu.pc += (i8)o;
}

// branch on overflow set
static void bvsrel(emu_t *e) {
    u8 o = pcnext(e);
    if ((e->cpu.p & V) == V) e->cpu.pc += (i8)o;
}

// clear carry flag
static inline void clc(emu_t *e) { e->cpu.p &= ~C; }
static void clcimp(emu_t *e) { clc(e); }

// clear decimal mode
static inline void cld(emu_t *e) { e->cpu.p &= ~D; }
static void cldimp(emu_t *e) { cld(e); }

// clear interrupt disable bit
static inline void cli(emu_t *e) { e->cpu.p &= ~I; }
static void cliimp(emu_t *e) { cli(e); }

// clear overflow flag
static inline void clv(emu_t *e) { e->cpu.p &= ~V; }
static void clvimp(emu_t *e) { clv(e); }

// compare memory with accumulator
static inline void cmp(emu_t *e, u8 m) {
    u8 r = e->cpu.a - m;
    u8 c = (e->cpu.a >= m) & 1;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= ZMASK(r) | NMASK(r) | c;
}
static void cmpimm(emu_t *e) { cmp(e, imm(e)); }
static void cmpzpg(emu_t *e) { cmp(e, zpg(e)); }
static void cmpzpx(emu_t *e) { cmp(e, zpx(e)); }
static void cmpabs(emu_t *e) { cmp(e, ast(e)); }
static void cmpabx(emu_t *e) { cmp(e, abx(e)); }
static void cmpaby(emu_t *e) { cmp(e, aby(e)); }
static void cmpxin(emu_t *e) { cmp(e, xin(e)); }
static void cmpyin(emu_t *e) { cmp(e, yin(e)); }

// compare memory and index x
static inline void cpx(emu_t *e, u8 m) {
    u8 r = e->cpu.x - m;
    u8 c = (e->cpu.x >= m) & 1;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= ZMASK(r) | NMASK(r) | c;
}
static void cpximm(emu_t *e) { cpx(e, imm(e)); }
static void cpxzpg(emu_t *e) { cpx(e, zpg(e)); }
static void cpxabs(emu_t *e) { cpx(e, ast(e)); }

// compare memory and index y
static inline void cpy(emu_t *e, u8 m) {
    u8 r = e->cpu.y - m;
    u8 c = (e->cpu.y >= m) & 1;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= ZMASK(r) | NMASK(r) | c;
}
static void cpyimm(emu_t *e) { cpy(e, imm(e)); }
static void cpyzpg(emu_t *e) { cpy(e, zpg(e)); }
static void cpyabs(emu_t *e) { cpy(e, ast(e)); }

// decrement memory by one
static void dec(emu_t *e, u8 *m) {
    (*m)--;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(*m) | NMASK(*m);
}
static void deczpg(emu_t *e) { dec(e, rzpg(e)); }
static void deczpx(emu_t *e) { dec(e, rzpx(e)); }
static void decabs(emu_t *e) { dec(e, rast(e)); }
static void decabx(emu_t *e) { dec(e, rabx(e)); }

// decrement index x by one
static void dex(emu_t *e) {
    e->cpu.x--;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.x) | NMASK(e->cpu.x);
}
static void deximp(emu_t *e) { dex(e); }

// decrement index y by one
static void dey(emu_t *e) {
    e->cpu.y--;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.y) | NMASK(e->cpu.y);
}
static void deyimp(emu_t *e) { dey(e); }

// exclusize-or memory with accumulator
static void eor(emu_t *e, u8 m) {
    e->cpu.a ^= m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}
static void eorimm(emu_t *e) { eor(e, imm(e)); }
static void eorzpg(emu_t *e) { eor(e, zpg(e)); }
static void eorzpx(emu_t *e) { eor(e, zpx(e)); }
static void eorabs(emu_t *e) { eor(e, ast(e)); }
static void eorabx(emu_t *e) { eor(e, abx(e)); }
static void eoraby(emu_t *e) { eor(e, aby(e)); }
static void eorxin(emu_t *e) { eor(e, xin(e)); }
static void eoryin(emu_t *e) { eor(e, yin(e)); }

// increment memory by one
static void inc(emu_t *e, u8 *m) {
    (*m)++;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(*m) | NMASK(*m);
}
static void inczpg(emu_t *e) { inc(e, rzpg(e)); }
static void inczpx(emu_t *e) { inc(e, rzpx(e)); }
static void incabs(emu_t *e) { inc(e, rast(e)); }
static void incabx(emu_t *e) { inc(e, rabx(e)); }

// increment x by one
static void inximp(emu_t *e) {
    e->cpu.x++;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.x) | NMASK(e->cpu.x);
}

// increment y by one
static void inyimp(emu_t *e) {
    e->cpu.y++;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.y) | NMASK(e->cpu.y);
}

// jump to new location
static void jmp(emu_t *e, u16 a) { e->cpu.pc = a; }
static void jmpabs(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    jmp(e, addr(lo, hi));
}
static void jmpind(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    jmp(e, addrat(e, addr(lo, hi)));
}

// jump to new location saving return address
static void jsrabs(emu_t *e) {
    u8 lo = pcnext(e);
    u8 hi = pcnext(e);
    u16 a = e->cpu.pc + 2;
    push(e, (a >> 8) & 0xFF);
    push(e, a & 0xFF);
    push(e, e->cpu.p | B | O);
    e->cpu.pc = addr(lo, hi);
}

// load accumulator with memory
static void lda(emu_t *e, u8 m) {
    e->cpu.a = m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}
static void ldaimm(emu_t *e) { lda(e, imm(e)); }
static void ldazpg(emu_t *e) { lda(e, zpg(e)); }
static void ldazpx(emu_t *e) { lda(e, zpx(e)); }
static void ldaabs(emu_t *e) { lda(e, ast(e)); }
static void ldaabx(emu_t *e) { lda(e, abx(e)); }
static void ldaaby(emu_t *e) { lda(e, aby(e)); }
static void ldaxin(emu_t *e) { lda(e, xin(e)); }
static void ldayin(emu_t *e) { lda(e, yin(e)); }

// load index x with memory
static void ldx(emu_t *e, u8 m) {
    e->cpu.x = m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.x) | NMASK(e->cpu.x);
}
static void ldximm(emu_t *e) { ldx(e, imm(e)); }
static void ldxzpg(emu_t *e) { ldx(e, zpg(e)); }
static void ldxzpy(emu_t *e) { ldx(e, zpy(e)); }
static void ldxabs(emu_t *e) { ldx(e, ast(e)); }
static void ldxaby(emu_t *e) { ldx(e, aby(e)); }

// load index y with memory
static void ldy(emu_t *e, u8 m) {
    e->cpu.y = m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.y) | NMASK(e->cpu.y);
}
static void ldyimm(emu_t *e) { ldy(e, imm(e)); }
static void ldyzpg(emu_t *e) { ldy(e, zpg(e)); }
static void ldyzpx(emu_t *e) { ldy(e, zpx(e)); }
static void ldyabs(emu_t *e) { ldy(e, ast(e)); }
static void ldyabx(emu_t *e) { ldy(e, abx(e)); }

static void lsr(emu_t *e, u8 *m) {
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= *m & C;
    *m >>= 1;
    e->cpu.p |= ZMASK(*m);
}
static void lsrzpg(emu_t *e) { lsr(e, rzpg(e)); }
static void lsracc(emu_t *e) { lsr(e, racc(e)); }
static void lsrabs(emu_t *e) { lsr(e, rast(e)); }
static void lsrzpx(emu_t *e) { lsr(e, rzpx(e)); }
static void lsrabx(emu_t *e) { lsr(e, rabx(e)); }

// no operation
static void nopimp(emu_t *e) { UNUSED(e); }
static void badnop(emu_t *e) {
    UNUSED(e);
    u16 a = addrat(e, e->cpu.pc-1);
    ulog(UWARN, "executed illegal opcode '$%02X' at $'%04X'", e->mem[a], a);
}

// or memory with accumulator
static inline void ora(emu_t *e, u8 m) {
    e->cpu.a |= m;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}
static void oraimm(emu_t *e) { ora(e, imm(e)); }
static void orazpg(emu_t *e) { ora(e, zpg(e)); }
static void orazpx(emu_t *e) { ora(e, zpx(e)); }
static void oraabs(emu_t *e) { ora(e, ast(e)); }
static void oraabx(emu_t *e) { ora(e, abx(e)); }
static void oraaby(emu_t *e) { ora(e, aby(e)); }
static void oraxin(emu_t *e) { ora(e, xin(e)); }
static void orayin(emu_t *e) { ora(e, yin(e)); }

// push accumulator on stack
static void phaimp(emu_t *e) { push(e, e->cpu.a); }

// push processor status on stack
static void phpimp(emu_t *e) { push(e, e->cpu.p | B | O); }

// pull accumulator from stack
static void plaimp(emu_t *e) {
    e->cpu.a = pull(e);
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}

// pull processor status from stack
static void plpimp(emu_t *e) { e->cpu.p = pull(e) & ~(B|O); }

// rotate one bit left (memory or accumulator)
static void rol(emu_t *e, u8 *m) {
    u8 c = e->cpu.p & C;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= ((*m & B7) >> 7) & C;
    *m = (*m << 1) | c;
    e->cpu.p |= ZMASK(*m) | NMASK(*m);
}
static void rolacc(emu_t *e) { rol(e, racc(e)); }
static void rolzpg(emu_t *e) { rol(e, rzpg(e)); }
static void rolzpx(emu_t *e) { rol(e, rzpx(e)); }
static void rolabs(emu_t *e) { rol(e, rast(e)); }
static void rolabx(emu_t *e) { rol(e, rabx(e)); }

// rotate one bit right (memory or accumulator)
static void ror(emu_t *e, u8 *m) {
    u8 c = e->cpu.p & C;
    e->cpu.p &= ~(Z|N|C);
    e->cpu.p |= *m & C;
    *m = (*m >> 1) | (c << 7);
    e->cpu.p |= ZMASK(*m) | NMASK(*m);
}
static void roracc(emu_t *e) { ror(e, racc(e)); }
static void rorzpg(emu_t *e) { ror(e, rzpg(e)); }
static void rorzpx(emu_t *e) { ror(e, rzpx(e)); }
static void rorabs(emu_t *e) { ror(e, rast(e)); }
static void rorabx(emu_t *e) { ror(e, rabx(e)); }

// return from interrupt
static void rtiimp(emu_t *e) {
    e->cpu.p = pull(e) & ~(B|O);
    e->cpu.s = pull(e);
    e->cpu.s |= pull(e) << 8;
}

// return from subroutine
static void rtsimp(emu_t *e) {
    e->cpu.s = pull(e);
    e->cpu.s |= pull(e) << 8;
}

// subtract memory from accumulator with borrow
static inline void sbc(emu_t *e, u8 m) {
    i16 s = e->cpu.a - m - ((e->cpu.p & C) != C);
    u8 c = ((s >> 8) & C);
    e->cpu.p &= ~(Z|N|C|V);
    if (((e->cpu.a & B7) == (m & B7)) && ((s & B7) != (m & B7)))
        e->cpu.p |= V;
    e->cpu.a = (u8)(i8)s;
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a) | c;
}
static void sbcimm(emu_t *e) { sbc(e, imm(e)); }
static void sbczpg(emu_t *e) { sbc(e, zpg(e)); }
static void sbczpx(emu_t *e) { sbc(e, zpx(e)); }
static void sbcabs(emu_t *e) { sbc(e, ast(e)); }
static void sbcabx(emu_t *e) { sbc(e, abx(e)); }
static void sbcaby(emu_t *e) { sbc(e, aby(e)); }
static void sbcxin(emu_t *e) { sbc(e, xin(e)); }
static void sbcyin(emu_t *e) { sbc(e, yin(e)); }

// set carry flag
static void secimp(emu_t *e) { e->cpu.p |= C; }

// set decimal flag
static void sedimp(emu_t *e) { e->cpu.p |= D; }

// set interrupt disable status
static void seiimp(emu_t *e) { e->cpu.p |= I; }

// store accumulator in memory
static void sta(emu_t *e, u8 *m) { *m = e->cpu.a; }
static void stazpg(emu_t *e) { sta(e, rzpg(e)); }
static void stazpx(emu_t *e) { sta(e, rzpx(e)); }
static void staabs(emu_t *e) { sta(e, rast(e)); }
static void staabx(emu_t *e) { sta(e, rabx(e)); }
static void staaby(emu_t *e) { sta(e, raby(e)); }
static void staxin(emu_t *e) { sta(e, rxin(e)); }
static void stayin(emu_t *e) { sta(e, ryin(e)); }

// store index x in memory
static void stx(emu_t *e, u8 *m) { *m = e->cpu.x; }
static void stxzpg(emu_t *e) { stx(e, rzpg(e)); }
static void stxzpy(emu_t *e) { stx(e, rzpy(e)); }
static void stxabs(emu_t *e) { stx(e, rast(e)); }

// store index y in memory
static void sty(emu_t *e, u8 *m) { *m = e->cpu.y; }
static void styzpg(emu_t *e) { sty(e, rzpg(e)); }
static void styzpx(emu_t *e) { sty(e, rzpx(e)); }
static void styabs(emu_t *e) { sty(e, rast(e)); }

// transfer accumulator to index x
static void taximp(emu_t *e) {
    e->cpu.x = e->cpu.a;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.x) | NMASK(e->cpu.x);
}

// transfer accumulator to index y
static void tayimp(emu_t *e) {
    e->cpu.y = e->cpu.a;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.y) | NMASK(e->cpu.y);
}

// transfer stack pointer to index x
static void tsximp(emu_t *e) {
    e->cpu.x = e->cpu.s;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.x) | NMASK(e->cpu.x);
}

// transfer index x to accumulator
static void txaimp(emu_t *e) {
    e->cpu.a = e->cpu.x;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}

// transfer index x to stack register
static void txsimp(emu_t *e) { e->cpu.s = e->cpu.x; }

// transfer index y to accumulator
static void tyaimp(emu_t *e) {
    e->cpu.a = e->cpu.y;
    e->cpu.p &= ~(Z|N);
    e->cpu.p |= ZMASK(e->cpu.a) | NMASK(e->cpu.a);
}

// hardware interrupt
static void irq(emu_t *e) {
    push(e, (e->cpu.pc >> 8) & 0xFF);
    push(e, e->cpu.pc & 0xFF);
    push(e, e->cpu.p | O);
    e->cpu.pc = IRQ;
}

typedef void (*isn_t)(emu_t *e);
const isn_t insns[256] = {
/*           -0,     -1,     -2,     -3,     -4,     -5,     -6,     -7,     -8,     -9,     -A,     -B,     -C,     -D,     -E,     -F,       */
/* 0- */ brkimp, oraxin, badnop, badnop, badnop, orazpg, aslzpg, badnop, phpimp, oraimm, aslacc, badnop, badnop, oraabs, aslabs, badnop, /* 0- */
/* 1- */ bplrel, orayin, badnop, badnop, badnop, orazpx, aslzpx, badnop, clcimp, oraaby, badnop, badnop, badnop, oraabx, aslabx, badnop, /* 1- */
/* 2- */ jsrabs, andxin, badnop, badnop, bitzpg, andzpg, rolzpg, badnop, plpimp, andimm, rolacc, badnop, bitabs, andabs, rolabs, badnop, /* 2- */
/* 3- */ bmirel, andyin, badnop, badnop, badnop, andzpx, rolzpx, badnop, secimp, andaby, badnop, badnop, badnop, andabx, rolabx, badnop, /* 3- */
/* 4- */ rtiimp, eorxin, badnop, badnop, badnop, eorzpg, lsrzpg, badnop, phaimp, eorimm, lsracc, badnop, jmpabs, eorabs, lsrabs, badnop, /* 4- */
/* 5- */ bvcrel, eoryin, badnop, badnop, badnop, eorzpx, lsrzpx, badnop, cliimp, eoraby, badnop, badnop, badnop, eorabx, lsrabx, badnop, /* 5- */
/* 6- */ rtsimp, adcxin, badnop, badnop, badnop, adczpg, rorzpg, badnop, plaimp, adcimm, roracc, badnop, jmpind, adcabs, rorabs, badnop, /* 6- */
/* 7- */ bvsrel, adcyin, badnop, badnop, badnop, adczpx, rorzpx, badnop, seiimp, adcaby, badnop, badnop, badnop, adcabx, rorabx, badnop, /* 7- */
/* 8- */ badnop, staxin, badnop, badnop, styzpg, stazpg, stxzpg, badnop, deyimp, badnop, txaimp, badnop, styabs, staabs, stxabs, badnop, /* 8- */
/* 9- */ bccrel, stayin, badnop, badnop, styzpx, stazpx, stxzpy, badnop, tyaimp, staaby, txsimp, badnop, badnop, staabx, badnop, badnop, /* 9- */
/* A- */ ldyimm, ldaxin, ldximm, badnop, ldyzpg, ldazpg, ldxzpg, badnop, tayimp, ldaimm, taximp, badnop, ldyabs, ldaabs, ldxabs, badnop, /* A- */
/* B- */ bcsrel, ldayin, badnop, badnop, ldyzpx, ldazpx, ldxzpy, badnop, clvimp, ldaaby, tsximp, badnop, ldyabx, ldaabx, ldxaby, badnop, /* B- */
/* C- */ cpyimm, cmpxin, badnop, badnop, cpyzpg, cmpzpg, deczpg, badnop, inyimp, cmpimm, deximp, badnop, cpyabs, cmpabs, decabs, badnop, /* C- */
/* D- */ bnerel, cmpyin, badnop, badnop, badnop, cmpzpx, deczpx, badnop, cldimp, cmpaby, badnop, badnop, badnop, cmpabx, decabx, badnop, /* D- */
/* E- */ cpximm, sbcxin, badnop, badnop, cpxzpg, sbczpg, inczpg, badnop, inximp, sbcimm, nopimp, badnop, cpxabs, sbcabs, incabs, badnop, /* E- */
/* F- */ beqrel, sbcyin, badnop, badnop, badnop, sbczpx, inczpx, badnop, sedimp, sbcaby, badnop, badnop, badnop, sbcabx, incabx, badnop, /* F- */
/*           -0,     -1,     -2,     -3,     -4,     -5,     -6,     -7,     -8,     -9,     -A,     -B,     -C,     -D,     -E,     -F,       */
};

void emu_init(emu_t *e, const u8 *prog, int size) {
    UASSERT(e);
    UASSERT(prog);
    UASSERT(size == MEMSIZE);
    memset(e, 0, sizeof(*e));
    e->mem = umalloc(MEMSIZE);
    memcpy(e->mem, prog, size);
    e->cpu.pc = addrat(e, RES);
}

void emu_end(emu_t *e) {
    UASSERT(e);
    free(e->mem);
    memset(e, 0, sizeof(*e));
}

void emu_step(emu_t *e) {
    UASSERT(e);
    insns[pcnext(e)](e);
}

void emu_log_state(emu_t *e) {
    ulog(UINFO, "PC: %04X", e->cpu.pc);
    ulog(UINFO, "SR: %02X %c%c..%c%c%c%c", e->cpu.p,
         (e->cpu.p & N) ? 'N' : 'n',
         (e->cpu.p & V) ? 'V' : 'v',
         (e->cpu.p & D) ? 'D' : 'd',
         (e->cpu.p & I) ? 'I' : 'i',
         (e->cpu.p & Z) ? 'Z' : 'z',
         (e->cpu.p & C) ? 'C' : 'c');
    ulog(UINFO, "AC: %02X", e->cpu.a);
    ulog(UINFO, "XR: %02X", e->cpu.x);
    ulog(UINFO, "YR: %02X", e->cpu.y);
    ulog(UINFO, "SP: %02X", e->cpu.s);
}

void emu_log_next_insn(emu_t *e) {
    u8 opc = e->mem[e->cpu.pc];
    u8 lo = e->mem[(u16)e->cpu.pc+1];
    u8 hi = e->mem[(u16)e->cpu.pc+2];
    ulog(UINFO, "%s", asm_disassemble_opcode(opc, lo, hi));
}

