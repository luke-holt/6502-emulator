#include "emu.h"

#define UTIL_IMPL
#include "util.h"

int
main(int argc, char *argv[])
{
    ulog(UINFO, "wtf");

    u8 prog[MEMSIZE] = {
        [0] = 0xA2, // LDX
        [1] = 0x00, // #$00
        [2] = 0xE8, // INX
        [3] = 0xE0, // CPX #$10
        [4] = 0x10,
        [5] = 0xD0, // BEQ
        [6] = (i8)-4,
        [7] = 0x6C, // JMP (NMI)
        [8] = NMI & 0xFF,
        [9] = (NMI >> 8) & 0xFF,

        [NMI] = 0,
        [NMI+1] = 0,
        [RES] = 0,
        [RES+1] = 0,
        [IRQ] = 0,
        [IRQ+1] = 0,
    };

    emu_t e;
    emu_init(&e, prog, MEMSIZE);

    while (1) {
        emu_step(&e);
        emu_log_state(&e);
    }

    return 0;
}
