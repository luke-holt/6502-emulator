#include <string.h>

#include "insn.h"
 
#define UTIL_IMPL
#include "util.h"
 
void print_mode_info(void) {
    ulog(UNONE, "%-5s %-19s %-11s %s", "Abbr", "Mode", "Syntax", "Description");
    for (int i = 0; i < ARRLEN(modeinfo); i++) {
        ulog(UNONE, "%-5s %-19s %-11s %s", modeinfo[i][0], modeinfo[i][1], modeinfo[i][2], modeinfo[i][3]);
    }
}

void print_insn_info(u8 insn) {
    ulog(UNONE, "%s -> %s", insnnames[insn][0], insnnames[insn][1]);

    // 3, 19, 11, 114
    ulog(UNONE, "%-3s %-5s %-19s %-11s", "OPC", "Abbr", "Mode", "Syntax");

    // iter through insn map
    for (int opc = 0; opc < ARRLEN(insnmap); opc++) {
        // insn matches name
        if (insnmap[opc] == insn) {
            u8 m = modemap[opc];
            ulog(UNONE, "%02X  %-5s %-19s %-11s", opc, modeinfo[m][0], modeinfo[m][1], modeinfo[m][2]);
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
        ulog(UNONE, " -> `%s modes` for info on 6502 addressing modes", argv[0]);
        return 1;
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

    if (strcmp("modes", argv[1]) == 0) {
        print_mode_info();
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
