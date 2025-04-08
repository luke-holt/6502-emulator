#include <stdio.h>
#include <string.h>

#include "asm.h"

#include "asm.h"

#include "util.h"

typedef struct {
    size_t count;
    size_t capacity;
    char *items;
} str_t;


void
asm_assemble(const char *text, int size)
{

}

const char *
asm_disassemble(const u8 *bytecode, int size)
{
    str_t dasmly;
    da_init(&dasmly);

    int i = 0;
    u8 opc, lo, hi;
    const char *disasm;
    while (i < size) {
        opc = bytecode[0];
        lo = bytecode[1];
        hi = bytecode[2];

        disasm = asm_disassemble_opcode(opc, lo, hi);

        da_append_many(&dasmly, disasm, strlen(disasm));
        da_append(&dasmly, '\n');

        i += modelen[modemap[opc]];
    }

    // leak string
    return dasmly.items;
}

const char *asm_disassemble_opcode(u8 opc, u8 lo, u8 hi) {
    static char str[64];
    memset(str, 0, sizeof(str));

    // instruction name
    const char *name = insnnames[insnmap[opc]][0];

    switch (modemap[opc]) {
    case ACC: snprintf(str, sizeof(str), "%s %s", name, "A"); break;
    case ABS: snprintf(str, sizeof(str), "%s $%02X%02X", name, lo, hi); break;
    case ABX: snprintf(str, sizeof(str), "%s $%02X%02X,X", name, lo, hi); break;
    case ABY: snprintf(str, sizeof(str), "%s $%02X%02X,Y", name, lo, hi); break;
    case IMM: snprintf(str, sizeof(str), "%s #$%02X", name, lo); break;
    case IMP: snprintf(str, sizeof(str), "%s", name); break;
    case IND: snprintf(str, sizeof(str), "%s ($%02X%02X)", name, lo, hi); break;
    case XIN: snprintf(str, sizeof(str), "%s ($%02X,X)", name, lo); break;
    case YIN: snprintf(str, sizeof(str), "%s ($%02X),Y", name, lo); break;
    case REL: snprintf(str, sizeof(str), "%s $%02X", name, lo); break;
    case ZPG: snprintf(str, sizeof(str), "%s $%02X", name, lo); break;
    case ZPX: snprintf(str, sizeof(str), "%s $%02X,X", name, lo); break;
    case ZPY: snprintf(str, sizeof(str), "%s $%02X,Y", name, lo); break;
    case INV:
    default: snprintf(str, sizeof(str), "%02X -> invalid opcode", opc); break;
    }

    return str;
}
