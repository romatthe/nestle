use crate::cpu::opcodes::AddressingMode::{
    ABS, ABX, ABY, ACC, IDX, IDY, IMM, IMP, IND, REL, ZPG, ZPX, ZPY,
};
use crate::cpu::opcodes::Mnemonic::{
    ADC, AHX, ALR, ANC, AND, ARR, ASL, AXS, BCC, BCS, BEQ, BIT, BMI, BNE, BPL, BRK, BVC, BVS, CLC,
    CLD, CLI, CLV, CMP, CPX, CPY, DCP, DEC, DEX, DEY, EOR, INC, INX, INY, ISB, JMP, JSR, KIL, LAS,
    LAX, LDA, LDX, LDY, LSR, NOP, ORA, PHA, PHP, PLA, PLP, RLA, ROL, ROR, RRA, RTI, RTS, SAX, SBC,
    SEC, SED, SEI, SHX, SHY, SLO, SRE, STA, STX, STY, TAS, TAX, TAY, TSX, TXA, TXS, TYA, XAA,
};

/// Indicates the size of each instruction in bytes
pub const INSTRUCTION_SIZES: [u8; 256] = [
    2, 2, 0, 2, 2, 2, 2, 2, 1, 2, 1, 0, 3, 3, 3, 3, 2, 2, 0, 2, 2, 2, 2, 2, 1, 3, 1, 3, 3, 3, 3, 3,
    3, 2, 0, 2, 2, 2, 2, 2, 1, 2, 1, 0, 3, 3, 3, 3, 2, 2, 0, 2, 2, 2, 2, 2, 1, 3, 1, 3, 3, 3, 3, 3,
    1, 2, 0, 2, 2, 2, 2, 2, 1, 2, 1, 0, 3, 3, 3, 3, 2, 2, 0, 2, 2, 2, 2, 2, 1, 3, 1, 3, 3, 3, 3, 3,
    1, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0, 2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
    2, 2, 0, 2, 2, 2, 2, 2, 1, 0, 1, 0, 3, 3, 3, 3, 2, 2, 0, 0, 2, 2, 2, 2, 1, 3, 1, 0, 0, 3, 0, 0,
    2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 0, 3, 3, 3, 3, 2, 2, 0, 2, 2, 2, 2, 2, 1, 3, 1, 0, 3, 3, 3, 3,
    2, 2, 0, 2, 2, 2, 2, 2, 1, 2, 1, 0, 3, 3, 3, 3, 2, 2, 0, 2, 2, 2, 2, 2, 1, 3, 1, 3, 3, 3, 3, 3,
    2, 2, 0, 2, 2, 2, 2, 2, 1, 2, 1, 2, 3, 3, 3, 3, 2, 2, 0, 2, 2, 2, 2, 2, 1, 3, 1, 3, 3, 3, 3, 3,
];

/// Indicates the number of cycles used by each instruction
pub const INSTRUCTION_CYCLES: [u8; 256] = [
    7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4, 2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
    2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4, 2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
    2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
];

/// Indicates the number of cycles used by each instruction when a page is crossed
pub const INSTRUCTION_PAGE_CYCLES: [u8; 256] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
];

/// List of all illegal instructions
pub const INSTRUCTION_ILLEGAL: [u8; 76] = [
    0xEB, 0xC3, 0xC7, 0xCF, 0xD3, 0xD7, 0xDB, 0xDF, 0xE3, 0xE7, 0xEF, 0xF3, 0xF7, 0xFB, 0xFF, 0x03,
    0x07, 0x0F, 0x13, 0x17, 0x1B, 0x1F, 0x23, 0x27, 0x2F, 0x33, 0x37, 0x3B, 0x3F, 0x43, 0x47, 0x4F,
    0x53, 0x57, 0x5B, 0x5F, 0x63, 0x67, 0x6F, 0x73, 0x77, 0x7B, 0x7F, 0x1A, 0x3A, 0x5A, 0x7A, 0xDA,
    0xFA, 0x04, 0x14, 0x34, 0x44, 0x54, 0x64, 0x74, 0xD4, 0xF4, 0x80, 0x0C, 0x1C, 0x3C, 0x5C, 0x7C,
    0xDC, 0xFC, 0xA3, 0xA7, 0xAF, 0xB3, 0xB7, 0xBF, 0x83, 0x87, 0x8F, 0x97,
];

/// Indicates the addressing mode for each instruction
#[rustfmt::skip]
pub const INSTRUCTION_MODES: [AddressingMode; 256] = [
    IMP, IDX, IMP, IDX, ZPG, ZPG, ZPG, ZPG, IMP, IMM, ACC, IMM, ABS, ABS, ABS, ABS,
    REL, IDY, IMP, IDY, ZPX, ZPX, ZPX, ZPX, IMP, ABY, IMP, ABY, ABX, ABX, ABX, ABX,
    ABS, IDX, IMP, IDX, ZPG, ZPG, ZPG, ZPG, IMP, IMM, ACC, IMM, ABS, ABS, ABS, ABS,
    REL, IDY, IMP, IDY, ZPX, ZPX, ZPX, ZPX, IMP, ABY, IMP, ABY, ABX, ABX, ABX, ABX,
    IMP, IDX, IMP, IDX, ZPG, ZPG, ZPG, ZPG, IMP, IMM, ACC, IMM, ABS, ABS, ABS, ABS,
    REL, IDY, IMP, IDY, ZPX, ZPX, ZPX, ZPX, IMP, ABY, IMP, ABY, ABX, ABX, ABX, ABX,
    IMP, IDX, IMP, IDX, ZPG, ZPG, ZPG, ZPG, IMP, IMM, ACC, IMM, IND, ABS, ABS, ABS,
    REL, IDY, IMP, IDY, ZPX, ZPX, ZPX, ZPX, IMP, ABY, IMP, ABY, ABX, ABX, ABX, ABX,
    IMM, IDX, IMM, IDX, ZPG, ZPG, ZPG, ZPG, IMP, IMM, IMP, IMM, ABS, ABS, ABS, ABS,
    REL, IDY, IMP, IDY, ZPX, ZPX, ZPY, ZPY, IMP, ABY, IMP, ABY, ABX, ABX, ABY, ABY,
    IMM, IDX, IMM, IDX, ZPG, ZPG, ZPG, ZPG, IMP, IMM, IMP, IMM, ABS, ABS, ABS, ABS,
    REL, IDY, IMP, IDY, ZPX, ZPX, ZPY, ZPY, IMP, ABY, IMP, ABY, ABX, ABX, ABY, ABY,
    IMM, IDX, IMM, IDX, ZPG, ZPG, ZPG, ZPG, IMP, IMM, IMP, IMM, ABS, ABS, ABS, ABS,
    REL, IDY, IMP, IDY, ZPX, ZPX, ZPX, ZPX, IMP, ABY, IMP, ABY, ABX, ABX, ABX, ABX,
    IMM, IDX, IMM, IDX, ZPG, ZPG, ZPG, ZPG, IMP, IMM, IMP, IMM, ABS, ABS, ABS, ABS,
    REL, IDY, IMP, IDY, ZPX, ZPX, ZPX, ZPX, IMP, ABY, IMP, ABY, ABX, ABX, ABX, ABX,
];

/// Indicates the mnemonic of each instruction
#[rustfmt::skip]
pub const INSTRUCTION_MNEMONIC: [Mnemonic; 256] = [
    BRK, ORA, KIL, SLO, NOP, ORA, ASL, SLO,
    PHP, ORA, ASL, ANC, NOP, ORA, ASL, SLO,
    BPL, ORA, KIL, SLO, NOP, ORA, ASL, SLO,
    CLC, ORA, NOP, SLO, NOP, ORA, ASL, SLO,
    JSR, AND, KIL, RLA, BIT, AND, ROL, RLA,
    PLP, AND, ROL, ANC, BIT, AND, ROL, RLA,
    BMI, AND, KIL, RLA, NOP, AND, ROL, RLA,
    SEC, AND, NOP, RLA, NOP, AND, ROL, RLA,
    RTI, EOR, KIL, SRE, NOP, EOR, LSR, SRE,
    PHA, EOR, LSR, ALR, JMP, EOR, LSR, SRE,
    BVC, EOR, KIL, SRE, NOP, EOR, LSR, SRE,
    CLI, EOR, NOP, SRE, NOP, EOR, LSR, SRE,
    RTS, ADC, KIL, RRA, NOP, ADC, ROR, RRA,
    PLA, ADC, ROR, ARR, JMP, ADC, ROR, RRA,
    BVS, ADC, KIL, RRA, NOP, ADC, ROR, RRA,
    SEI, ADC, NOP, RRA, NOP, ADC, ROR, RRA,
    NOP, STA, NOP, SAX, STY, STA, STX, SAX,
    DEY, NOP, TXA, XAA, STY, STA, STX, SAX,
    BCC, STA, KIL, AHX, STY, STA, STX, SAX,
    TYA, STA, TXS, TAS, SHY, STA, SHX, AHX,
    LDY, LDA, LDX, LAX, LDY, LDA, LDX, LAX,
    TAY, LDA, TAX, LAX, LDY, LDA, LDX, LAX,
    BCS, LDA, KIL, LAX, LDY, LDA, LDX, LAX,
    CLV, LDA, TSX, LAS, LDY, LDA, LDX, LAX,
    CPY, CMP, NOP, DCP, CPY, CMP, DEC, DCP,
    INY, CMP, DEX, AXS, CPY, CMP, DEC, DCP,
    BNE, CMP, KIL, DCP, NOP, CMP, DEC, DCP,
    CLD, CMP, NOP, DCP, NOP, CMP, DEC, DCP,
    CPX, SBC, NOP, ISB, CPX, SBC, INC, ISB,
    INX, SBC, NOP, SBC, CPX, SBC, INC, ISB,
    BEQ, SBC, KIL, ISB, NOP, SBC, INC, ISB,
    SED, SBC, NOP, ISB, NOP, SBC, INC, ISB,
];

#[rustfmt::skip]
#[derive(PartialEq)]
pub enum AddressingMode {
    ABS,        // Absolute             Operand is an address and and both bytes are used,           ex: LDA $16A0
    ABX,        // Indexed Absolute X   Operand is 2-byte address, X register is added to it         eg: STA $1000,X
    ABY,        // Indexed Absolute Y   Operand is 2-byte address, Y register is added to it         eg: STA $1000,Y
    ACC,        // Accumulator          No operands, accumulator is implied,                         eg: ASL
    IMM,        // Immediate            Operand value is contained in instruction itself,            ex: LDA #$07
    IMP,        // Implied              No operands, addressing is implied by the instruction,       eg: TAX
    IDX,        // Indexed Indirect     2-byte pointer from 1-byte address and adding X register     eg: LDA ($40, X)
    IND,        // Indirect             Memory location is 2-byte pointer at adjacent locations      eg: JMP ($0020)
    IDY,        // Indirect Indexed     2-byte pointer from 1-byte address and adding Y after read   eg: LDA ($46), Y
    REL,        // Relative             1-byte signed operand is added to the program counter        eg: BEQ $04
    ZPG,        // ZeroPage             Operand is an address and only the low byte is used,         ex: LDA $EE
    ZPX,        // Indexed ZeroPage X   Operand is 1-byte address, X register is added to it         eg: STA $00,X
    ZPY,        // Indexed ZeroPage Y   Operand is 1-byte address, Y register is added to it         eg: STA $00,Y
    UNKNOWN
}

#[rustfmt::skip]
#[derive(Debug, PartialEq)]
pub enum Mnemonic {
    LDA, LDX, LDY, STA, STX, STY, TAX, TAY, TSX, TXA, TXS, TYA,     // Storage
    ADC, DEC, DEX, DEY, INC, INX, INY, SBC,                         // Math
    AND, ASL, BIT, EOR, LSR, ORA, ROL, ROR,                         // Bitwise
    BCC, BCS, BEQ, BMI, BNE, BPL, BVC, BVS,                         // Branch
    JMP, JSR, RTI, RTS,                                             // Jump
    CLC, CLD, CLI, CLV, CMP, CPX, CPY, SEC, SED, SEI,               // Registers
    PHA, PHP, PLA, PLP,                                             // Stack
    BRK, NOP,                                                       // System
    KIL, SLO, ANC, RLA, SRE, ALR, RRA, SAX, XAA, AHX, LAX, DCP,ISB, ARR, AXS, TAS, SHY, SHX, LAS, // ?
    UNKNOWN
}
