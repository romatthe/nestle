use crate::cpu::opcodes::Mnemonic;

pub mod opcodes;

// Structure representing the Status Register
// Some notes on the status register
// Ref: https://wiki.nesdev.com/w/index.php/CPU_status_flag_behavior
// 7  bit  0
// ---- ----
// NVss DIZC
// |||| ||||
// |||| |||+- Carry: 1 if last addition or shift resulted in a carry, or if last subtraction resulted in no borrow
// |||| ||+-- Zero: 1 if last operation resulted in a 0 value
// |||| |+--- Interrupt: Interrupt inhibit (0: /IRQ and /NMI get through; 1: only /NMI gets through)
// |||| +---- Decimal: 1 to make ADC and SBC use binary-coded decimal arithmetic (ignored on second-source 6502 like that in the NES)
// ||++------ s: No effect, used by the stack copy
// |+-------- Overflow: 1 if last ADC or SBC resulted in signed overflow, or D6 from last BIT
// +--------- Negative: Set to bit 7 of the last operation
bitflags::bitflags! {
    pub struct StatusRegister: u8 {
        const C = 0b0000_0001;  // Carry
        const Z = 0b0000_0010;  // Zero
        const I = 0b0000_0100;  // Interrupt
        const D = 0b0000_1000;  // Decimal
        const V = 0b0100_0000;  // Overflow
        const N = 0b1000_0000;  // Negative
    }
}

impl StatusRegister {
    fn new() -> Self {
        StatusRegister { bits: 0x34 }
    }
}

pub struct Registers {
    /// Accumulator
    pub a: u8,
    /// X register
    pub x: u8,
    /// Y register
    pub y: u8,
    /// Bitfield status register
    pub status: StatusRegister,
}

impl Registers {
    fn new() -> Self {
        Registers {
            a: 0,
            x: 0,
            y: 0,
            status: StatusRegister::new(),
        }
    }
}

pub struct Cpu {
    /// Program counter
    pub pc: u16,
    /// Stack pointer
    pub sp: u8,
    /// Registers
    pub regs: Registers,
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            pc: 0,
            sp: 0xfd,
            regs: Registers::new(),
        }
    }

    pub fn exec(&mut self, op: &Mnemonic) {
        match op {
            LDA => {}
        }
    }
}
