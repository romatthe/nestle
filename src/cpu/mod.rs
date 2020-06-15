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

    /// Returns the processor status flags
    fn get_flags(&self) -> u8 {
        let mut flags: u8 = 0;
        flags |= self.c << 0;
        flags |= self.z << 1;
        flags |= self.i << 2;
        flags |= self.d << 3;
        flags |= self.b << 4;
        flags |= self.v << 6;
        flags |= self.n << 7;

        flags
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

    fn push_u8(&mut self, value: u8) {
        let address = 0x100 + self.sp as u16;
        self.bus.write_u8(address, value);
        self.sp = self.sp.wrapping_sub(1);
    }

    fn push_u16(&mut self, value: u16) {
        self.push_u8((value >> 8) as u8);
        self.push_u8(value as u8);
    }

    fn pop_u8(&mut self) -> u8 {
        self.sp += 1;
        self.bus.read_u8(0x100 | (self.sp as u16))
    }

    fn pop_u16(&mut self) -> u16 {
        let low = self.pop_u8() as u16;
        let high = self.pop_u8() as u16;
        high <<8 | low
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
            Mnemonic::BCC => self.bcc(mode),
            Mnemonic::BCS => self.bcs(mode),
            Mnemonic::BEQ => self.beq(mode),
            Mnemonic::BIT => self.bit(mode),
            Mnemonic::BMI => self.bmi(mode),
            Mnemonic::BNE => self.bne(mode),
            Mnemonic::BPL => self.bpl(mode),
            Mnemonic::BRK => self.brk(mode),
            Mnemonic::BVC => self.bvc(mode),
            Mnemonic::BVS => self.bvs(mode),
            Mnemonic::CLC => self.clc(),
            Mnemonic::CLD => self.cld(),
            Mnemonic::CLI => self.cli(),
            Mnemonic::CLV => self.clv(),
            Mnemonic::CMP => self.cmp(mode),
            Mnemonic::CPX => self.cpx(mode),
            Mnemonic::CPY => self.cpy(mode),
            Mnemonic::DEC => self.dec(mode),
            Mnemonic::DEX => self.dex(),
            Mnemonic::DEY => self.dey(),
            Mnemonic::EOR => self.eor(mode),
            Mnemonic::INC => self.inc(mode),
            Mnemonic::INX => self.inx(),
            Mnemonic::INY => self.iny(),
            Mnemonic::JMP => self.jmp(mode),
            Mnemonic::JSR => self.jsr(mode),
            Mnemonic::LDA => self.lda(mode),
            Mnemonic::LDX => self.ldx(mode),
            Mnemonic::LDY => self.ldy(mode),
            Mnemonic::LSR => self.lsr(mode),
            Mnemonic::NOP => self.nop(),
            Mnemonic::ORA => self.ora(mode),
            Mnemonic::PHA => self.pha(),
            Mnemonic::PHP => self.php(mode),
            Mnemonic::PLA => self.pla(mode),
            Mnemonic::PLP => self.plp(mode),
            Mnemonic::ROL => self.rol(mode),
            Mnemonic::ROR => self.ror(mode),
            Mnemonic::RTI => self.rti(),
            Mnemonic::RTS => self.rts(),
            Mnemonic::SEI => self.sei(mode),
            _ => {}
        }
    }

    fn compare(&mut self, a: u8, b: u8) {
        self.regs.set_zn(a - b);
        if a >= b {
            self.regs.c = 1;
        } else {
            self.regs.c = 0;
        }
    }

    /// Add with carry
    fn adc(&mut self, mode: &AddressingMode) {
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
    fn and(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.regs.a = self.regs.a & operand;
        self.regs.set_zn(self.regs.a);
    }

    /// Arithmetic Shift Left
    fn asl(&mut self, mode: &AddressingMode) {
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

    /// Branch if carry clear
    fn bcc(&mut self, mode: &AddressingMode) {
        if self.regs.c == 0 {
            self.pc = self.get_operand_address(mode);
        }
    }

    /// Branch if carry set
    fn bcs(&mut self, mode: &AddressingMode) {
        if self.regs.c != 0 {
            self.pc = self.get_operand_address(mode);
        }
    }

    /// Branch if equal
    fn beq(&mut self, mode: &AddressingMode) {
        if self.regs.z != 0 {
            self.pc = self.get_operand_address(mode);
        }
    }

    /// Bit test
    fn bit(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.regs.v = (operand >> 6) & 1;
        self.regs.set_z(operand & self.regs.a);
        self.regs.set_n(operand);
    }

    /// Branch if minus
    fn bmi(&mut self, mode: &AddressingMode) {
        if self.regs.n != 0 {
            self.pc = self.get_operand_address(mode);
        }
    }

    /// Branch if not equal
    fn bne(&mut self, mode: &AddressingMode) {
        if self.regs.z == 0 {
            self.pc = self.get_operand_address(mode);
        }
    }

    /// Branch if positive
    fn bpl(&mut self, mode: &AddressingMode) {
        if self.regs.n == 0 {
            self.pc = self.get_operand_address(mode);
        }
    }

    /// Force interrupt
    fn brk(&mut self, mode: &AddressingMode) {
        self.push_u16(self.pc);
        self.php(mode);
        self.sei(mode);
        self.pc = self.bus.read_u16(0xFFFE);
    }

    /// Branch if overflow clear
    fn bvc(&mut self, mode: &AddressingMode) {
        if self.regs.v == 0 {
            self.pc = self.get_operand_address(mode);
        }
    }

    /// Branch if overflow set
    fn bvs(&mut self, mode: &AddressingMode) {
        if self.regs.v != 0 {
            self.pc = self.get_operand_address(mode);
        }
    }

    /// Clear Carry Flag
    fn clc(&mut self) {
        self.regs.c = 0;
    }

    /// Clear decimal mode
    fn cld(&mut self) {
        self.regs.d = 0;
    }

    /// Clear interrupt disable
    fn cli(&mut self) {
        self.regs.i = 0;
    }

    /// Clear overflow flag
    fn clv(&mut self) {
        self.regs.v = 0;
    }

    // Compare
    fn cmp(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.compare(self.regs.a, operand);
    }

    /// Compare X Register
    fn cpx(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.compare(self.regs.x, operand);
    }

    /// Compare X Register
    fn cpy(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.compare(self.regs.y, operand);
    }

    /// Decrement memory
    fn dec(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.bus.write_u8(address, operand);
        self.regs.set_zn(operand);
    }

    /// Decrement x register
    fn dex(&mut self) {
        self.regs.x -= 1;
        self.regs.set_zn(self.regs.x);
    }

    /// Decrement y register
    fn dey(&mut self) {
        self.regs.y -= 1;
        self.regs.set_zn(self.regs.y);
    }

    /// Exclusive or
    fn eor(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.regs.a = self.regs.a ^ operand;
        self.regs.set_zn(self.regs.a);
    }

    /// Increment memory
    fn inc(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address) + 1;

        self.bus.write_u8(address, operand);
        self.regs.set_zn(self.regs.a);
    }

    /// Increment x register
    fn inx(&mut self) {
        self.regs.x += 1;
        self.regs.set_zn(self.regs.x);
    }

    /// Increment y register
    fn iny(&mut self) {
        self.regs.y += 1;
        self.regs.set_zn(self.regs.y);
    }

    /// Jump
    fn jmp(&mut self, mode: &AddressingMode) {
        self.pc = self.get_operand_address(mode);
    }

    /// Jump to subroutine
    fn jsr(&mut self, mode: &AddressingMode) {
        self.push_u16(self.pc - 1);
        self.pc = self.get_operand_address(mode);
    }

    /// Load accumulator
    fn lda(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.regs.a = operand;
    }

    /// Load y register
    fn ldx(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.regs.x = operand;
        self.regs.set_zn(self.regs.x);
    }

    /// Load y register
    fn ldy(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let operand = self.bus.read_u8(address);

        self.regs.y = operand;
        self.regs.set_zn(self.regs.y);
    }

    /// Logical shift right
    fn lsr(&mut self, mode: &AddressingMode) {
        if *mode == AddressingMode::ACC {
            self.regs.c = self.regs.a & 1;
            self.regs.a >>= 1;
            self.regs.set_zn(self.regs.a);
        } else {
            let address = self.get_operand_address(mode);
            let mut operand = self.bus.read_u8(address);

            self.regs.c = operand & 1;
            operand >>= 1;

            self.bus.write_u8(address, operand);
            self.regs.set_zn(operand);
        }
    }

    /// No operation
    fn nop(&self) {

    }

    /// Logical inclusive or
    fn ora(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        self.regs.a = self.regs.a | self.bus.read_u8(address);
        self.regs.set_zn(self.regs.a);
    }

    /// Push Accumulator
    fn pha(&mut self) {
        self.push_u8(self.regs.a)
    }


    /// Push processor status
    fn php(&mut self, mode: &AddressingMode) {
        self.push_u8(self.regs.get_flags() | 0x10);
    }

    /// Pull Accumulator
    fn pla(&mut self, mode: &AddressingMode) {
        self.regs.a = self.pop_u8();
        self.regs.set_zn(self.regs.a);
    }

    /// Pull processor status
    fn plp(&mut self, mode: &AddressingMode) {
        let results = self.pop_u8() & 0xEF | 0x20;
        self.regs.set_flags(results);
    }

    /// Rotate left
    fn rol(&mut self, mode: &AddressingMode) {
        let c = self.regs.c;

        if *mode == AddressingMode::ACC {
            self.regs.c = (self.regs.a >> 7) & 1;
            self.regs.a = (self.regs.a << 1) | c;
            self.regs.set_zn(self.regs.a);
        } else {
            let address = self.get_operand_address(mode);
            let mut operand = self.bus.read_u8(address);

            self.regs.c = (operand >> 7) & 1;
            operand = (operand << 1) | c;
            self.bus.write_u8(address, operand);
            self.regs.set_zn(operand);
        }
    }

    /// Rotate right
    fn ror(&mut self, mode: &AddressingMode) {
        let c = self.regs.c;

        if *mode == AddressingMode::ACC {
            self.regs.c = self.regs.a  & 1;
            self.regs.a = (self.regs.a >> 1) | (c << 7);
            self.regs.set_zn(self.regs.a);
        } else {
            let address = self.get_operand_address(mode);
            let mut operand = self.bus.read_u8(address);

            operand = (operand >> 1) | (c << 7);
            self.bus.write_u8(address, operand);
            self.regs.set_zn(operand);
        }
    }

    /// Return from interrupt
    fn rti(&mut self) {
        let result = self.pop_u8() & 0xEF | 0x20;
        self.regs.set_flags(result);
        self.pc = self.pop_u16();
    }

    /// Return from subroutine
    fn rts(&mut self) {
        self.pc = self.pop_u16() + 1;
    }

    /// Set interrupt disable
    fn sei(&mut self, mode: &AddressingMode) {
        self.regs.i = 1;
    }
}
