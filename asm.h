#ifndef ASM_H
#define ASM_H

#include "insn.h"

void asm_assemble(const char *text, int size);
const char *asm_disassemble(const u8 *bytecode, int size);
const char *asm_disassemble_opcode(u8 opc, u8 lo, u8 hi);

#endif // ASM_H
