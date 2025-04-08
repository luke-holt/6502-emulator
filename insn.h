#ifndef INSN_H
#define INSN_H

typedef unsigned char u8;

typedef enum {
    ADC = 0x00, AND = 0x01, ASL = 0x02, BCC = 0x03,
    BCS = 0x04, BEQ = 0x05, BIT = 0x06, BMI = 0x07,
    BNE = 0x08, BPL = 0x09, BRK = 0x0A, BVC = 0x0B,
    BVS = 0x0C, CLC = 0x0D, CLD = 0x0E, CLI = 0x0F,
    CLV = 0x10, CMP = 0x11, CPX = 0x12, CPY = 0x13,
    DEC = 0x14, DEX = 0x15, DEY = 0x16, EOR = 0x17,
    INC = 0x18, INX = 0x19, INY = 0x1A, JMP = 0x1B,
    JSR = 0x1C, LDA = 0x1D, LDX = 0x1E, LDY = 0x1F,
    LSR = 0x20, NOP = 0x21, ORA = 0x22, PHA = 0x23,
    PHP = 0x24, PLA = 0x25, PLP = 0x26, ROL = 0x27,
    ROR = 0x28, RTI = 0x29, RTS = 0x2A, SBC = 0x2B,
    SEC = 0x2C, SED = 0x2D, SEI = 0x2E, STA = 0x2F,
    STX = 0x30, STY = 0x31, TAX = 0x32, TAY = 0x33,
    TSX = 0x34, TXA = 0x35, TXS = 0x36, TYA = 0x37,
    INSN_COUNT
} insn_t;

typedef enum {
    ACC = 0x0, ABS = 0x1, ABX = 0x2, ABY = 0x3,
    IMM = 0x4, IMP = 0x5, IND = 0x6, XIN = 0x7,
    YIN = 0x8, REL = 0x9, ZPG = 0xA, ZPX = 0xB,
    ZPY = 0xC, INV = 0xD,
    AMODE_COUNT
} amode_t;

static const char *insnnames[INSN_COUNT][2] = {
    [ADC] = {"ADC", "add with carry"},
    [AND] = {"AND", "and (with accumulator)"},
    [ASL] = {"ASL", "arithmetic shift left"},
    [BCC] = {"BCC", "branch on carry clear"},
    [BCS] = {"BCS", "branch on carry set"},
    [BEQ] = {"BEQ", "branch on equal (zero set)"},
    [BIT] = {"BIT", "bit test"},
    [BMI] = {"BMI", "branch on minus (negative set)"},
    [BNE] = {"BNE", "branch on not equal (zero clear)"},
    [BPL] = {"BPL", "branch on plus (negative clear)"},
    [BRK] = {"BRK", "break / interrupt"},
    [BVC] = {"BVC", "branch on overflow clear"},
    [BVS] = {"BVS", "branch on overflow set"},
    [CLC] = {"CLC", "clear carry"},
    [CLD] = {"CLD", "clear decimal"},
    [CLI] = {"CLI", "clear interrupt disable"},
    [CLV] = {"CLV", "clear overflow"},
    [CMP] = {"CMP", "compare (with accumulator)"},
    [CPX] = {"CPX", "compare with X"},
    [CPY] = {"CPY", "compare with Y"},
    [DEC] = {"DEC", "decrement"},
    [DEX] = {"DEX", "decrement X"},
    [DEY] = {"DEY", "decrement Y"},
    [EOR] = {"EOR", "exclusive or (with accumulator)"},
    [INC] = {"INC", "increment"},
    [INX] = {"INX", "increment X"},
    [INY] = {"INY", "increment Y"},
    [JMP] = {"JMP", "jump"},
    [JSR] = {"JSR", "jump subroutine"},
    [LDA] = {"LDA", "load accumulator"},
    [LDX] = {"LDX", "load X"},
    [LDY] = {"LDY", "load Y"},
    [LSR] = {"LSR", "logical shift right"},
    [NOP] = {"NOP", "no operation"},
    [ORA] = {"ORA", "or with accumulator"},
    [PHA] = {"PHA", "push accumulator"},
    [PHP] = {"PHP", "push processor status (SR)"},
    [PLA] = {"PLA", "pull accumulator"},
    [PLP] = {"PLP", "pull processor status (SR)"},
    [ROL] = {"ROL", "rotate left"},
    [ROR] = {"ROR", "rotate right"},
    [RTI] = {"RTI", "return from interrupt"},
    [RTS] = {"RTS", "return from subroutine"},
    [SBC] = {"SBC", "subtract with carry"},
    [SEC] = {"SEC", "set carry"},
    [SED] = {"SED", "set decimal"},
    [SEI] = {"SEI", "set interrupt disable"},
    [STA] = {"STA", "store accumulator"},
    [STX] = {"STX", "store X"},
    [STY] = {"STY", "store Y"},
    [TAX] = {"TAX", "transfer accumulator to X"},
    [TAY] = {"TAY", "transfer accumulator to Y"},
    [TSX] = {"TSX", "transfer stack pointer to X"},
    [TXA] = {"TXA", "transfer X to accumulator"},
    [TXS] = {"TXS", "transfer X to stack pointer"},
    [TYA] = {"TYA", "transfer Y to accumulator"},
};

// abbreviated, un-abbreviated, syntax, byte count, description
static const char *modeinfo[AMODE_COUNT][5] = {
    [ACC] = {"A",     "Accumulator",         "OPC A",       "1", "operand is AC (implied single byte instruction)"},
    [ABS] = {"abs",   "absolute",            "OPC $LLHH",   "3", "operand is address $HHLL"},
    [ABX] = {"abs,X", "absolute, X-indexed", "OPC $LLHH,X", "3", "operand is address; effective address is address incremented by X with carry"},
    [ABY] = {"abs,Y", "absolute, Y-indexed", "OPC $LLHH,Y", "3", "operand is address; effective address is address incremented by Y with carry"},
    [IMM] = {"#",     "immediate",           "OPC #$BB",    "2", "operand is byte BB"},
    [IMP] = {"impl",  "implied",             "OPC",         "1", "operand implied"},
    [IND] = {"ind",   "indirect",            "OPC ($LLHH)", "3", "operand is address; effective address is contents of word at address: C.w($HHLL)"},
    [XIN] = {"X,ind", "X-indexed, indirect", "OPC ($LL,X)", "2", "operand is zeropage address; effective address is word in (LL + X, LL + X + 1), inc. without carry: C.w($00LL + X)"},
    [YIN] = {"ind,Y", "indirect, Y-indexed", "OPC ($LL),Y", "2", "operand is zeropage address; effective address is word in (LL, LL + 1) incremented by Y with carry: C.w($00LL) + Y"},
    [REL] = {"rel",   "relative",            "OPC $BB",     "2", "branch target is PC + signed offset BB"},
    [ZPG] = {"zpg",   "zeropage",            "OPC $LL",     "2", "operand is zeropage address (hi-byte is zero, address = $00LL)"},
    [ZPX] = {"zpg,X", "zeropage, X-indexed", "OPC $LL,X",   "2", "operand is zeropage address; effective address is address incremented by X without carry"},
    [ZPY] = {"zpg,Y", "zeropage, Y-indexed", "OPC $LL,Y",   "2", "operand is zeropage address; effective address is address incremented by Y without carry"},
    [INV] = {"n/a",   "invalid",             "n/a",         "0", "invalid addressing mode"},
};

static const u8 modelen[AMODE_COUNT] = {
    [ACC] = 1, [ABS] = 3, [ABX] = 3, [ABY] = 3,
    [IMM] = 2, [IMP] = 1, [IND] = 3, [XIN] = 2,
    [YIN] = 2, [REL] = 2, [ZPG] = 2, [ZPX] = 2,
    [ZPY] = 2, [INV] = 0,
};

static const u8 modemap[256] = {
/*        -0,  -1,  -2,  -3,  -4,  -5,  -6,  -7,  -8,  -9,  -A,  -B,  -C,  -D,  -E,  -F,       */
/* 0- */ IMP, XIN, INV, INV, INV, ZPG, ZPG, INV, IMP, IMM, ACC, INV, INV, ABS, ABS, INV, /* 0- */
/* 1- */ REL, YIN, INV, INV, INV, ZPX, ZPX, INV, IMP, ABY, INV, INV, INV, ABX, ABX, INV, /* 1- */
/* 2- */ ABS, XIN, INV, INV, ZPG, ZPG, ZPG, INV, IMP, IMM, ACC, INV, ABS, ABS, ABS, INV, /* 2- */
/* 3- */ REL, YIN, INV, INV, INV, ZPX, ZPX, INV, IMP, ABY, INV, INV, INV, ABX, ABX, INV, /* 3- */
/* 4- */ IMP, XIN, INV, INV, INV, ZPG, ZPG, INV, IMP, IMM, ACC, INV, ABS, ABS, ABS, INV, /* 4- */
/* 5- */ REL, YIN, INV, INV, INV, ZPX, ZPX, INV, IMP, ABY, INV, INV, INV, ABX, ABX, INV, /* 5- */
/* 6- */ IMP, XIN, INV, INV, INV, ZPG, ZPG, INV, IMP, IMM, ACC, INV, IND, ABS, ABS, INV, /* 6- */
/* 7- */ REL, YIN, INV, INV, INV, ZPX, ZPX, INV, IMP, ABY, INV, INV, INV, ABX, ABX, INV, /* 7- */
/* 8- */ INV, XIN, INV, INV, ZPG, ZPG, ZPG, INV, IMP, INV, IMP, INV, ABS, ABS, ABS, INV, /* 8- */
/* 9- */ REL, YIN, INV, INV, ZPX, ZPX, ZPY, INV, IMP, ABY, IMP, INV, INV, ABX, INV, INV, /* 9- */
/* A- */ IMM, XIN, IMM, INV, ZPG, ZPG, ZPG, INV, IMP, IMM, IMP, INV, ABS, ABS, ABS, INV, /* A- */
/* B- */ REL, YIN, INV, INV, ZPX, ZPX, ZPY, INV, IMP, ABY, IMP, INV, ABX, ABX, ABY, INV, /* B- */
/* C- */ IMM, XIN, INV, INV, ZPG, ZPG, ZPG, INV, IMP, IMM, IMP, INV, ABS, ABS, ABS, INV, /* C- */
/* D- */ REL, YIN, INV, INV, INV, ZPX, ZPX, INV, IMP, ABY, INV, INV, INV, ABX, ABX, INV, /* D- */
/* E- */ IMM, XIN, INV, INV, ZPG, ZPG, ZPG, INV, IMP, IMM, IMP, INV, ABS, ABS, ABS, INV, /* E- */
/* F- */ REL, YIN, INV, INV, INV, ZPX, ZPX, INV, IMP, ABY, INV, INV, INV, ABX, ABX, INV, /* F- */
/*        -0,  -1,  -2,  -3,  -4,  -5,  -6,  -7,  -8,  -9,  -A,  -B,  -C,  -D,  -E,  -F,       */
};

static const int insnmap[256] = {
/*        -0,  -1,  -2,  -3,  -4,  -5,  -6,  -7,  -8,  -9,  -A,  -B,  -C,  -D,  -E,  -F,       */
/* 0- */ BRK, ORA, NOP, NOP, NOP, ORA, ASL, NOP, PHP, ORA, ASL, NOP, NOP, ORA, AND, NOP, /* 0- */
/* 1- */ BPL, ORA, NOP, NOP, NOP, ORA, ASL, NOP, CLC, ORA, NOP, NOP, NOP, ORA, AND, NOP, /* 1- */
/* 2- */ JSR, AND, NOP, NOP, BIT, AND, ROL, NOP, PLP, AND, ROL, NOP, BIT, AND, ROL, NOP, /* 2- */
/* 3- */ BMI, AND, NOP, NOP, NOP, AND, ROL, NOP, SEC, AND, NOP, NOP, NOP, AND, ROL, NOP, /* 3- */
/* 4- */ RTI, EOR, NOP, NOP, NOP, EOR, LSR, NOP, PHA, EOR, LSR, NOP, JMP, EOR, LSR, NOP, /* 4- */
/* 5- */ BVC, EOR, NOP, NOP, NOP, EOR, LSR, NOP, CLI, EOR, NOP, NOP, NOP, EOR, LSR, NOP, /* 5- */
/* 6- */ RTS, ADC, NOP, NOP, NOP, ADC, ROR, NOP, PLA, ADC, ROR, NOP, JMP, ADC, ROR, NOP, /* 6- */
/* 7- */ BVS, ADC, NOP, NOP, NOP, ADC, ROR, NOP, SEI, ADC, NOP, NOP, NOP, ADC, ROR, NOP, /* 7- */
/* 8- */ NOP, STA, NOP, NOP, STY, STA, STY, NOP, DEY, NOP, TXA, NOP, STY, STA, STY, NOP, /* 8- */
/* 9- */ BCC, STA, NOP, NOP, STY, STA, STY, NOP, TYA, STA, TXS, NOP, NOP, STA, NOP, NOP, /* 9- */
/* A- */ LDY, LDA, LDX, NOP, LDY, LDA, LDX, NOP, TAY, LDA, TAX, NOP, LDY, LDA, LDX, NOP, /* A- */
/* B- */ BCS, LDA, NOP, NOP, LDY, LDA, LDX, NOP, CLV, LDA, TSX, NOP, LDY, LDA, LDX, NOP, /* B- */
/* C- */ CPY, CMP, NOP, NOP, CPY, CMP, DEC, NOP, INY, CMP, DEX, NOP, CPY, CMP, DEC, NOP, /* C- */
/* D- */ BNE, CMP, NOP, NOP, NOP, CMP, DEC, NOP, CLD, CMP, NOP, NOP, NOP, CMP, DEC, NOP, /* D- */
/* E- */ CPX, SBC, NOP, NOP, CPX, SBC, INC, NOP, INX, SBC, NOP, NOP, CPX, SBC, INC, NOP, /* E- */
/* F- */ BEQ, SBC, NOP, NOP, NOP, SBC, INC, NOP, SED, SBC, NOP, NOP, NOP, SBC, INC, NOP, /* F- */
/*        -0,  -1,  -2,  -3,  -4,  -5,  -6,  -7,  -8,  -9,  -A,  -B,  -C,  -D,  -E,  -F,       */
};

#endif // INSN_H
