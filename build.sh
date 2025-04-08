#!/usr/bin/env sh

set -x

gcc -o emulator asm.c emu.c main.c -Wall -pedantic
gcc -o insnhelp insnhelp.c -Wall -pedantic
gcc -o assembler asm.c assembler.c -Wall -pedantic
