use crate::cpu::opcodes::{Mnemonic, AddressingMode};
use crate::cpu::bus::Bus;

pub mod bus;
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
    /// Carry
    c: u8,
    /// Zero
    z: u8,
    /// Interrupt
    i: u8,
    /// Decimal
    d: u8,
    /// Break
    b: u8,
    /// Overflow
    v: u8,
    /// Negative
    n: u8,
}

impl Registers {
    fn new() -> Self {
        let mut regs = Registers { a: 0, x: 0, y: 0, c: 0, z: 0, i: 0, d: 0, b: 0, v: 0, n: 0 };
        regs.set_flags(0x24);

        regs
    }

    fn set_flags(&mut self, flags: u8) {
        self.c = (flags >> 0) & 1;
        self.z = (flags >> 1) & 1;
        self.i = (flags >> 2) & 1;
        self.d = (flags >> 3) & 1;
        self.b = (flags >> 4) & 1;
        self.v = (flags >> 6) & 1;
        self.n = (flags >> 7) & 1;
    }

    fn set_zn(&mut self, value: u8) {
        self.set_z(value);
        self.set_n(value);
    }

    fn set_z(&mut self, value: u8) {
        if value == 0 {
            self.z = 1;
        } else {
            self.z = 0;
        }
    }

    fn set_n(&mut self, value: u8) {
        if value & 0x80 != 0 {
            self.n = 1;
        } else {
            self.n = 0;
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
    /// System bus
    pub bus: Bus,
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            pc: 0,
            sp: 0xfd,
            regs: Registers::new(),
            bus: Bus::new(),
        }
    }

    pub fn step(&mut self) {
        let opcode = self.bus.read_u8(self.pc);
        let mode = &opcodes::INSTRUCTION_MODES[opcode as usize];
        let mnemonic = &opcodes::INSTRUCTION_MNEMONIC[opcode as usize];

        // let operand_address = self.operand_address(opcode, mode);
        // let operand = self.bus.read_u8(operand_address);

        self.exec(opcode, mnemonic, mode);
    }

    fn get_operand_address(&mut self, mode: &AddressingMode) -> u16 {
        match mode {
            // Absolute addressing
            AddressingMode::ABS => self.bus.read_u16(self.pc + 1),
            // Indexed Absolute X addressing
            AddressingMode::ABX => {
                self.bus.read_u16(self.pc.wrapping_add(1)) + (self.regs.x as u16)
            }
            // Indexed Absolute Y addressing
            AddressingMode::ABY => {
                self.bus.read_u16(self.pc.wrapping_add(1)) + (self.regs.y as u16)
            }
            // Accumulator addressing
            AddressingMode::ACC => 0,
            // Immediate addressing
            AddressingMode::IMM => self.pc.wrapping_add(1),
            // Implied addressing
            AddressingMode::IMP => 0,
            // Indexed Indirect addressing
            AddressingMode::IDX => self.bus.read_u16(
                self.bus.read_u8(self.pc.wrapping_add(1)) as u16 + self.regs.x as u16,
            ),
            // Indirect addressing
            AddressingMode::IND => self.bus.read_u16(self.bus.read_u16(self.pc.wrapping_add(1))),
            // Indirect Indexed addressing
            AddressingMode::IDY => self.bus.read_u16(
                self.bus.read_u8(self.pc.wrapping_add(1)) as u16 + self.regs.y as u16,
            ),
            // Relative addressing
            AddressingMode::REL => {
                let offset = self.bus.read_u8(self.pc.wrapping_add(1)) as u16;
                if offset < 0x80 {
                    self.pc + 2 + offset
                } else {
                    self.pc + 2 + offset - 0x100
                }
            }
            // Zero Page addressing
            AddressingMode::ZPG => self.bus.read_u8(self.pc.wrapping_add(1)) as u16,
            // Indexed Zero Page X addressing
            AddressingMode::ZPX => {
                ((self.bus.read_u8(self.pc.wrapping_add(1)) + self.regs.x) as u16) & 0xff
            }
            // Indexed Zero Page Y addressing
            AddressingMode::ZPY => {
                ((self.bus.read_u8(self.pc.wrapping_add(1)) + self.regs.y) as u16) & 0xff
            }
            // Unknown
            AddressingMode::UNKNOWN => panic!("Unknown addressing mode encountered"),
        }
    }

    fn exec(&mut self, opcode: u8, mnemonic: &Mnemonic, mode: &AddressingMode) {
        match mnemonic {
            Mnemonic::ADC => self.adc(mode),
            Mnemonic::AND => self.and(mode),
            Mnemonic::LDA => self.lda(mode),

            _ => {}
        }
    }

    /// Add with carry
    pub fn adc(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        let a = self.regs.a;
        let c = self.regs.c;

        self.regs.a = a + operand + c;
        self.regs.set_zn(self.regs.a);

        if (a as i32) + (operand as i32) + (c as i32) > 0xFF {
            self.regs.c = 1;
        } else {
            self.regs.c = 0;
        }

        if (a ^ operand) & 0x80 == 0 && (a ^ self.regs.a) & 0x80 != 0 {
            self.regs.v = 1;
        } else {
            self.regs.v = 0;
        }
    }

    /// Logical AND
    pub fn and(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.regs.a = self.regs.a & operand;
        self.regs.set_zn(self.regs.a);
    }

    /// Arithmetic Shift Left
    pub fn asl(&mut self, mode: &AddressingMode) {
        if *mode == AddressingMode::ACC {
            self.regs.c = (self.regs.a >> 7) & 1;
            self.regs.a <<= 1;
            self.regs.set_zn(self.regs.a);
        } else {
            let address = self.get_operand_address(mode);
            let operand = self.bus.read_u8(address);
            let value = operand << 1;

            self.regs.c = (operand >> 7) & 1;
            self.bus.write_u8(address, value);
            self.regs.set_zn(value);
        }
    }

    /// Load accumulator
    pub fn lda(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.regs.a = operand;
    }
}
