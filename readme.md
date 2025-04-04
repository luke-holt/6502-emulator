# Overview
The 6502 Emulation Project is an adventure into emulation, assembly programming and machine code, and assembler design.

## Emulator
The emulator interprets 6502 bytecode and behaves as a 6502 processor would.

## Instruction Helper
The instruction helper command is useful for instruction assembly format, addressing modes, descriptions and machine code representation. It's used to support emulator and assembler development.
```sh
$ ./insnhelp
usage: ./insnhelp <insn>
 -> `./insnhelp TAX` for details on the TAX instruction
 -> `./insnhelp all` for info on all 6502 instructions
 -> `./insnhelp modes` for info on 6502 addressing modes
```

For example, to get information on the "load X register" instruction.
```sh
$ ./insnhelp LDX
LDX -> load X
Abbr  Mode                Syntax      Bytes Description
#     immediate           OPC #$BB    2     operand is byte BB
zpg   zeropage            OPC $LL     2     operand is zeropage address (hi-byte is zero, address = $00LL)
abs   absolute            OPC $LLHH   3     operand is address $HHLL
zpg,Y zeropage, Y-indexed OPC $LL,Y   2     operand is zeropage address; effective address is address incremented by Y without carry
abs,Y absolute, Y-indexed OPC $LLHH,Y 3     operand is address; effective address is address incremented by Y with carry
```

## Assembler
WIP.

