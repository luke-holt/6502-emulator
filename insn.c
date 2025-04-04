#include <string.h>

#include "insn.h"
 
#define UTIL_IMPL
#include "util.h"
 
void print_insn_info(u8 insn) {
    ulog(UNONE, "%s -> %s", insnnames[insn][0], insnnames[insn][1]);

    // 3, 19, 11, 114
    ulog(UNONE, "%-3s %-6s %-19s %-11s", "OPC", "Abbr", "Mode", "Syntax");

    // iter through insn map
    for (int opc = 0; opc < ARRLEN(insnmap); opc++) {
        // insn matches name
        if (insnmap[opc] == insn) {
            u8 m = modemap[opc];
            ulog(UNONE, "%02X  %-6s %-19s %-11s", opc, modenames[m][0], modenames[m][1], modenames[m][2]);
        }
    }
}

int insn_from_name(const char *name) {
    for (int i = 0; i < ARRLEN(insnnames); i++) {
        if (strcmp(name, insnnames[i][0]) == 0) {
            return i;
        }
    }
    return -1;
}

int
main(int argc, char *argv[])
{
    if ((argc == 1) || (argc != 2)) {
        ulog(UNONE, "usage: %s <insn>", argv[0]);
        ulog(UNONE, " -> `%s TAX` for details on the TAX instruction", argv[0]);
        ulog(UNONE, " -> `%s all` for info on all 6502 instructions", argv[0]);
        return 0;
    }

    if (strcmp("all", argv[1]) == 0) {
        // iter through insn names
        for (int i = 0; i < ARRLEN(insnnames); i++) {
            print_insn_info(i);

            if (i < (ARRLEN(insnnames) - 1))
                ulog(UNONE, "");
        }
        return 0;
    }

    int insn = insn_from_name(argv[1]);
    if (insn < 0) {
        ulog(UFATL, "Unkown instruction `%s`", argv[1]);
        return 0;
    }

    print_insn_info(insn);

    return 0;
}
