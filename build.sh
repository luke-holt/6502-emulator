#!/usr/bin/env sh

gcc -o emu emu.c main.c -Wall -pedantic
gcc -o insn insn.c -Wall -pedantic
