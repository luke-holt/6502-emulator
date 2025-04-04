#!/usr/bin/env sh

gcc -o emu emu.c main.c -Wall -pedantic
gcc -o insnhelp insnhelp.c -Wall -pedantic
