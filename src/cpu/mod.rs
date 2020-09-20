use crate::cpu::opcodes::{Mnemonic, AddressingMode, INSTRUCTION_SIZES};
use crate::cpu::bus::Bus;
use log::warn;
use std::collections::HashMap;
use std::fmt;
use bitflags::_core::fmt::Formatter;

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
    /// Break, bit 2
    u: u8,
    /// Overflow
    v: u8,
    /// Negative
    n: u8,
}

impl Registers {
    fn new() -> Self {
        let mut regs = Registers { a: 0, x: 0, y: 0, c: 0, z: 0, i: 0, d: 0, b: 0, u: 0, v: 0, n: 0 };
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
        flags |= self.u << 5;
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
        self.u = (flags >> 5) & 1;
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

const STACK_OFFSET: u16 = 0x0100;
const STACK_RESET: u8 = 0xfd;

pub struct Cpu {
    /// Program counter
    pub pc: u16,
    /// Stack pointer
    pub sp: u8,
    /// Registers
    pub regs: Registers,
    /// RAM
    memory: [u8; 0xFFFF],
    /// CPU should exit when set to `false`
    running: bool,
    // Mark if branched, and if so, which address is being branched to
    // branched: Option<u16>,
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            pc: 0,
            sp: STACK_RESET,
            regs: Registers::new(),
            memory: [0x0; 0xFFFF],
            running: true,
        }
    }

    pub fn run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();

        while self.running {
            self.step();
        }
    }

    fn step(&mut self) {
        let opcode = self.mem_read(self.pc);
        let mode = &opcodes::INSTRUCTION_MODES[opcode as usize];
        let mnemonic = &opcodes::INSTRUCTION_MNEMONIC[opcode as usize];

        self.exec(opcode, mnemonic, mode);
    }

    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut Cpu),
    {
        while self.running {
            let opcode = self.mem_read(self.pc);
            let mode = &opcodes::INSTRUCTION_MODES[opcode as usize];
            let mnemonic = &opcodes::INSTRUCTION_MNEMONIC[opcode as usize];

            self.exec(opcode, mnemonic, mode);

            // Execute the callback
            callback(self);
        }
    }

    pub fn reset(&mut self) {
        // Reset the registers
        self.regs.a = 0;
        self.regs.x = 0;
        self.regs.y = 0;
        self.regs.set_flags(0b100100);
        self.sp = STACK_RESET;

        // Set the program counter to the start of the program as stored in 0xFFFC during load
        self.pc = self.mem_read_u16(0xFFFC);
    }

    pub fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    pub fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    pub fn mem_read_u16(&self, addr: u16) -> u16 {
        u16::from_le_bytes([self.mem_read(addr), self.mem_read(addr + 1)])
    }

    pub fn mem_write_u16(&mut self, addr: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xFF) as u8;
        self.mem_write(addr, lo);
        self.mem_write(addr + 1, hi);
    }

    /// Push a byte unto the stack
    fn push_u8(&mut self, value: u8) {
        self.mem_write(STACK_OFFSET as u16 + self.sp as u16, value);
        self.sp = self.sp.wrapping_sub(1);
    }

    /// Push two bytes unto the stack
    fn push_u16(&mut self, value: u16) {
        self.push_u8((value >> 8) as u8);
        self.push_u8((value & 0xFF) as u8);
    }

    /// Pop a byte off the stack
    fn pop_u8(&mut self) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        self.mem_read(STACK_OFFSET as u16 + self.sp as u16)
    }

    /// Pop two bytes off the stack
    fn pop_u16(&mut self) -> u16 {
        let lo = self.pop_u8() as u16;
        let hi = self.pop_u8() as u16;
        hi << 8 | lo
    }

    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x0600 .. (0x0600 + program.len())].copy_from_slice(&program[..]);

        // Store the reference to the start of the code in 0xFFFC
        self.mem_write_u16(0xFFFC, 0x0600);
    }

    /// Emulates a 6502 bug that caused the low byte to wrap without incrementing the high byte
    fn bug_mem_read_wraparound(&self, address: u16) -> u16 {
        let a = address;
        let b = (a & 0xFF00) | ((a as u8).wrapping_add(1) as u16);
        let lo= self.mem_read(a);
        let hi= self.mem_read(b);

        (hi as u16) << 8 | (lo as u16)
    }

    /// Get the address of the next operand based on the addressing mode
    fn get_next_operand_address(&self, mode: &AddressingMode) -> u16 {
        // We need to look at the next byte after the current position of the program counter
        let pc_next = self.pc.wrapping_add(1);

        match mode {
            // Absolute addressing
            AddressingMode::ABS => self.mem_read_u16(pc_next),
            // Indexed Absolute X addressing
            AddressingMode::ABX => self.mem_read_u16(pc_next).wrapping_add(self.regs.x as u16),
            // Indexed Absolute Y addressing
            AddressingMode::ABY => self.mem_read_u16(pc_next).wrapping_add(self.regs.y as u16),
            // Accumulator addressing
            AddressingMode::ACC => 0,
            // Immediate addressing
            AddressingMode::IMM => pc_next,
            // Implied addressing
            AddressingMode::IMP => 0,
            // Indexed Indirect addressing
            AddressingMode::IDX => {
                let target = self.mem_read(pc_next).wrapping_add(self.regs.x) as u16;
                // Emulate a 6502 wraparound bug
                let address_bugged = self.bug_mem_read_wraparound(target);

                address_bugged
            },
            // Indirect addressing
            AddressingMode::IND => {
                let target = self.mem_read_u16(pc_next);
                // Emulate a 6502 wraparound bug
                let address_bugged = self.bug_mem_read_wraparound(target);

                address_bugged
            },
            // Indirect Indexed addressing
            AddressingMode::IDY => {
                let base = self.mem_read(pc_next);

                let lo = self.mem_read(base as u16);
                let hi = self.mem_read(base.wrapping_add(1) as u16);
                let deref_base = u16::from_le_bytes([lo, hi]);
                let deref = deref_base.wrapping_add(self.regs.y as u16);

                deref
            },
            // Relative addressing
            AddressingMode::REL => {
                let offset = self.mem_read(pc_next) as u16;
                if offset < 0x80 {
                    pc_next + 1 + offset
                } else {
                    pc_next + 1 + offset - 0x100
                }
            },
            // Zero Page addressing
            AddressingMode::ZPG => self.mem_read(pc_next) as u16,
            // Indexed Zero Page X addressing
            AddressingMode::ZPX => self.mem_read(pc_next).wrapping_add(self.regs.x) as u16,
            // Indexed Zero Page Y addressing
            AddressingMode::ZPY => self.mem_read(pc_next).wrapping_add(self.regs.y) as u16,
            // Unknown
            AddressingMode::UNKNOWN => panic!("Unknown addressing mode encountered"),
        }
    }

    /// Execute an instruction in the current CPU state
    fn exec(&mut self, opcode: u8, mnemonic: &Mnemonic, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let size = &opcodes::INSTRUCTION_SIZES[opcode as usize];

        self.pc += *size as u16;

        match mnemonic {
            // Variants
            Mnemonic::ASL if opcode != 0x0A => self.asl(address),
            Mnemonic::ASL if opcode == 0x0A => self.asl_acc(),
            Mnemonic::JMP if opcode != 0x4C => self.jmp(address),
            Mnemonic::JMP if opcode == 0x4C => self.jmp_abs(address),
            Mnemonic::LSR if opcode != 0x4A => self.lsr(address),
            Mnemonic::LSR if opcode == 0x4A => self.lsr_acc(),
            Mnemonic::ROL if opcode != 0x2A => self.rol(address),
            Mnemonic::ROL if opcode == 0x2A => self.rol_acc(),
            Mnemonic::ROR if opcode != 0x6A => self.ror(address),
            Mnemonic::ROR if opcode == 0x6A => self.ror_acc(),

            // Regular
            Mnemonic::ADC => self.adc(address),
            Mnemonic::AND => self.and(address),
            Mnemonic::BCC => self.bcc(address),
            Mnemonic::BCS => self.bcs(address),
            Mnemonic::BEQ => self.beq(address),
            Mnemonic::BIT => self.bit(address),
            Mnemonic::BMI => self.bmi(address),
            Mnemonic::BNE => self.bne(address),
            Mnemonic::BPL => self.bpl(address),
            Mnemonic::BRK => self.brk(),
            Mnemonic::BVC => self.bvc(address),
            Mnemonic::BVS => self.bvs(address),
            Mnemonic::CLC => self.clc(),
            Mnemonic::CLD => self.cld(),
            Mnemonic::CLI => self.cli(),
            Mnemonic::CLV => self.clv(),
            Mnemonic::CMP => self.cmp(address),
            Mnemonic::CPX => self.cpx(address),
            Mnemonic::CPY => self.cpy(address),
            Mnemonic::DEC => self.dec(address),
            Mnemonic::DEX => self.dex(),
            Mnemonic::DEY => self.dey(),
            Mnemonic::EOR => self.eor(address),
            Mnemonic::INC => self.inc(address),
            Mnemonic::INX => self.inx(),
            Mnemonic::INY => self.iny(),
            Mnemonic::JSR => self.jsr(address),
            Mnemonic::LDA => self.lda(address),
            Mnemonic::LDX => self.ldx(address),
            Mnemonic::LDY => self.ldy(address),
            Mnemonic::NOP => self.nop(),
            Mnemonic::ORA => self.ora(address),
            Mnemonic::PHA => self.pha(),
            Mnemonic::PHP => self.php(),
            Mnemonic::PLA => self.pla(),
            Mnemonic::PLP => self.plp(),
            Mnemonic::RTI => self.rti(),
            Mnemonic::RTS => self.rts(),
            Mnemonic::SBC => self.sbc(address),
            Mnemonic::SEC => self.sec(),
            Mnemonic::SED => self.sed(),
            Mnemonic::SEI => self.sei(),
            Mnemonic::STA => self.sta(address),
            Mnemonic::STX => self.stx(address),
            Mnemonic::STY => self.sty(address),
            Mnemonic::TAX => self.tax(),
            Mnemonic::TAY => self.tay(),
            Mnemonic::TSX => self.tsx(),
            Mnemonic::TXA => self.txa(),
            Mnemonic::TXS => self.txs(),
            Mnemonic::TYA => self.tya(),

            // Illegal
            Mnemonic::AHX => self.ahx(),
            Mnemonic::ALR => self.alr(),
            Mnemonic::ANC => self.anc(),
            Mnemonic::ARR => self.arr(),
            Mnemonic::AXS => self.axs(),
            Mnemonic::DCP => self.dcp(address),
            Mnemonic::ISB => self.isb(address),
            Mnemonic::KIL => self.kil(),
            Mnemonic::LAS => self.las(),
            Mnemonic::LAX => self.lax(address),
            Mnemonic::RLA => self.rla(),
            Mnemonic::RRA => self.rra(),
            Mnemonic::SAX => self.sax(address),
            Mnemonic::SHX => self.shx(),
            Mnemonic::SHY => self.shy(),
            Mnemonic::SLO => self.slo(),
            Mnemonic::SRE => self.sre(),
            Mnemonic::TAS => self.tas(),
            Mnemonic::XAA => self.xaa(),

            // Unknown
            _ => panic!("Unknown opcode encountered: {:#X?}", opcode),
        }
    }

    /// Compare values and set the zero and carry flags as appropriate
    fn compare(&mut self, compare_with: u8, other: u8) {
        self.regs.set_zn(compare_with.wrapping_sub(other));

        if other <= compare_with {
            self.regs.c = 1;
        } else {
            self.regs.c = 0;
        }
    }

    /// Add with carry
    fn adc(&mut self, addr: u16) {
        let operand = self.mem_read(addr);

        // Store initial values for later reference
        let a = self.regs.a;
        let c = self.regs.c;

        let value = a as u16 + operand as u16 + c as u16;
        let result = value as u8;

        self.regs.a = result;
        self.regs.set_zn(self.regs.a);

        // Check if carry flag must be set
        if value > 0xFF {
            self.regs.c = 1;
        } else {
            self.regs.c = 0;
        }

        // Check if overflow flag must be set
        if (a ^ operand) & 0x80 == 0 && (a ^ self.regs.a) & 0x80 != 0 {
            self.regs.v = 1;
        } else {
            self.regs.v = 0;
        }
    }

    /// Logical AND
    fn and(&mut self, addr: u16) {
        let operand = self.mem_read(addr);

        self.regs.a = self.regs.a & operand;
        self.regs.set_zn(self.regs.a);
    }

    /// Arithmetic Shift Left
    fn asl(&mut self, addr: u16) {
        let operand = self.mem_read(addr);
        let value = operand << 1;

        self.regs.c = (operand >> 7) & 1;
        self.mem_write(addr, value);
        self.regs.set_zn(value);
    }

    /// Arithmetic Shift Left with accumulator addressing mode
    fn asl_acc(&mut self) {
        // Set the carry flag accordingly
        self.regs.c = (self.regs.a >> 7) & 1;

        self.regs.a <<= 1;
        self.regs.set_zn(self.regs.a);
    }

    /// Branch if carry clear
    fn bcc(&mut self, addr: u16) {
        if self.regs.c == 0 {
            self.pc = addr;
        }
    }

    /// Branch if carry set
    fn bcs(&mut self, addr: u16) {
        if self.regs.c == 1 {
            self.pc = addr;
        }
    }

    /// Branch if equal
    fn beq(&mut self, addr: u16) {
        if self.regs.z == 1 {
            self.pc = addr;
        }
    }

    /// Bit test
    fn bit(&mut self, addr: u16) {
        let operand = self.mem_read(addr);

        self.regs.v = (operand >> 6) & 1;
        self.regs.set_z(operand & self.regs.a);
        self.regs.set_n(operand);
    }

    /// Branch if minus
    fn bmi(&mut self, addr: u16) {
        if self.regs.n == 1 {
            self.pc = addr;
        }
    }

    /// Branch if not equal
    fn bne(&mut self, addr: u16) {
        if self.regs.z == 0 {
            self.pc = addr;
        }
    }

    /// Branch if positive
    fn bpl(&mut self, addr: u16) {
        if self.regs.n == 0 {
            self.pc = addr;
        }
    }

    /// Force interrupt
    fn brk(&mut self) {
        // TODO: Figure out the correct way to handle interrupts
    }

    /// Branch if overflow clear
    fn bvc(&mut self, addr: u16) {
        if self.regs.v == 0 {
            self.pc = addr;
        }
    }

    /// Branch if overflow set
    fn bvs(&mut self, addr: u16) {
        if self.regs.v == 1 {
            self.pc = addr;
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

    /// Compare
    fn cmp(&mut self, addr: u16) {
        let operand = self.mem_read(addr);

        self.compare(self.regs.a, operand);
    }

    /// Compare X Register
    fn cpx(&mut self, addr: u16) {
        let operand = self.mem_read(addr);

        self.compare(self.regs.x, operand);
    }

    /// Compare X Register
    fn cpy(&mut self, addr: u16) {
        let operand = self.mem_read(addr);

        self.compare(self.regs.y, operand);
    }

    /// Decrement memory
    fn dec(&mut self, addr: u16) {
        let operand = self.mem_read(addr).wrapping_sub(1);

        self.mem_write(addr, operand);
        self.regs.set_zn(operand);
    }

    /// Decrement x register
    fn dex(&mut self) {
        self.regs.x = self.regs.x.wrapping_sub(1);
        self.regs.set_zn(self.regs.x);
    }

    /// Decrement y register
    fn dey(&mut self) {
        self.regs.y = self.regs.y.wrapping_sub(1);
        self.regs.set_zn(self.regs.y);
    }

    /// Exclusive or
    fn eor(&mut self, addr: u16) {
        let operand = self.mem_read(addr);

        self.regs.a = self.regs.a ^ operand;
        self.regs.set_zn(self.regs.a);
    }

    /// Increment memory
    fn inc(&mut self, addr: u16) {
        let operand = self.mem_read(addr).wrapping_add(1);

        self.mem_write(addr, operand);
        self.regs.set_zn(operand);
    }

    /// Increment x register
    fn inx(&mut self) {
        self.regs.x = self.regs.x.wrapping_add(1);
        self.regs.set_zn(self.regs.x);
    }

    /// Increment y register
    fn iny(&mut self) {
        self.regs.y = self.regs.y.wrapping_add(1);
        self.regs.set_zn(self.regs.y);
    }

    /// Jump
    fn jmp(&mut self, addr: u16) {
        self.pc = addr;
    }

    /// Jump with absolute addressing
    fn jmp_abs(&mut self, addr: u16) {
        self.pc = addr;
    }

    /// Jump to subroutine
    fn jsr(&mut self, addr: u16) {
        self.push_u16(self.pc - 1);
        self.pc = addr;
    }

    /// Load accumulator
    fn lda(&mut self, addr: u16) {
        let operand = self.mem_read(addr);
        self.regs.a = operand;
        self.regs.set_zn(self.regs.a);
    }

    /// Load x register
    fn ldx(&mut self, addr: u16) {
        let operand = self.mem_read(addr);
        self.regs.x = operand;
        self.regs.set_zn(self.regs.x);
    }

    /// Load y register
    fn ldy(&mut self, addr: u16) {
        let operand = self.mem_read(addr);

        self.regs.y = operand;
        self.regs.set_zn(self.regs.y);
    }

    /// Logical shift right
    fn lsr(&mut self, addr: u16) {
        let mut operand = self.mem_read(addr);

        self.regs.c = operand & 1;
        operand >>= 1;

        self.mem_write(addr, operand);
        self.regs.set_zn(operand);
    }

    /// Logical shift right with accumulator addressing mode
    fn lsr_acc(&mut self) {
        self.regs.c = self.regs.a & 1;
        self.regs.a >>= 1;
        self.regs.set_zn(self.regs.a);
    }

    /// No operation
    fn nop(&self) {

    }

    /// Logical inclusive or
    fn ora(&mut self, addr: u16) {
        let operand = self.mem_read(addr);

        self.regs.a = self.regs.a | operand;
        self.regs.set_zn(self.regs.a);
    }

    /// Push Accumulator
    fn pha(&mut self) {
        self.push_u8(self.regs.a);
    }

    /// Push processor status
    fn php(&mut self) {
        // http://wiki.nesdev.com/w/index.php/Status_flags
        let flags = self.regs.get_flags() | 0b00110000;
        self.push_u8(flags);
    }

    /// Pull Accumulator
    fn pla(&mut self) {
        self.regs.a = self.pop_u8();
        self.regs.set_zn(self.regs.a);
    }

    /// Pull processor status
    fn plp(&mut self) {
        let flags = self.pop_u8();
        self.regs.set_flags(flags);
        self.regs.b = 0;
        self.regs.u = 1;
    }

    /// Rotate left
    fn rol(&mut self, addr: u16) {
        let operand = self.mem_read(addr);
        let c = self.regs.c;
        self.regs.c = (operand >> 7) & 1;
        let operand = (operand << 1) | c;
        self.mem_write(addr, operand);
        self.regs.set_zn(operand);
    }

    /// Rotate left with accumulator addressing
    fn rol_acc(&mut self) {
        let c = self.regs.c;
        self.regs.c = (self.regs.a >> 7) & 1;
        self.regs.a = (self.regs.a << 1) | c;
        self.regs.set_zn(self.regs.a);
    }

    /// Rotate right
    fn ror(&mut self, addr: u16) {
        let c = self.regs.c;
        let mut operand = self.mem_read(addr);

        self.regs.c = operand & 1;
        let operand = (operand >> 1) | (c << 7);
        self.mem_write(addr, operand);
        self.regs.set_zn(operand);
    }

    /// Rotate right with accumulator addressing
    fn ror_acc(&mut self) {
        let c = self.regs.c;
        self.regs.c = self.regs.a & 1;
        self.regs.a = (self.regs.a >> 1) | (c << 7);
        self.regs.set_zn(self.regs.a);
    }

    /// Return from interrupt
    fn rti(&mut self) {
        // Pull the status flags from the stack
        let flags = self.pop_u8();
        self.regs.set_flags(flags);
        self.regs.b = 0;
        self.regs.u = 1;

        // Pull the program counter from the stack
        self.pc = self.pop_u16();
    }

    /// Return from subroutine
    fn rts(&mut self) {
        self.pc = self.pop_u16() + 1;
    }

    /// Subtract with Carry
    fn sbc(&mut self, addr: u16) {
        let operand = self.mem_read(addr);
        let data = (operand as i8).wrapping_neg().wrapping_sub(1) as u8;

        let a = self.regs.a;
        let c = self.regs.c;

        let value = a as u16 + data as u16 + c as u16;
        let result = value as u8;

        self.regs.a = result;
        self.regs.set_zn(self.regs.a);

        // Check if carry flag must be set
        if value > 0xFF {
            self.regs.c = 1;
        } else {
            self.regs.c = 0;
        }

        // Check if overflow flag must be set
        if (a ^ data) & 0x80 == 0 && (a ^ self.regs.a) & 0x80 != 0 {
            self.regs.v = 1;
        } else {
            self.regs.v = 0;
        }
    }

    /// Set carry flag
    fn sec(&mut self) {
        self.regs.c = 1;
    }

    /// Set decimal flag
    fn sed(&mut self) {
        self.regs.d = 1;
    }

    /// Set interrupt disable
    fn sei(&mut self) {
        self.regs.i = 1;
    }

    /// Store accumulator
    fn sta(&mut self, addr: u16) {
        self.mem_write(addr, self.regs.a);
    }

    /// Store the x register
    fn stx(&mut self, addr: u16) {
        self.mem_write(addr, self.regs.x);
    }

    /// Store the y register
    fn sty(&mut self, addr: u16) {
        self.mem_write(addr, self.regs.y);
    }

    /// Transfer accumulator to x
    fn tax(&mut self) {
        self.regs.x = self.regs.a;
        self.regs.set_zn(self.regs.x);
    }

    /// Transfer accumulator to y
    fn tay(&mut self) {
        self.regs.y = self.regs.a;
        self.regs.set_zn(self.regs.y);
    }

    /// Transfer stack pointer to x
    fn tsx(&mut self) {
        self.regs.x = self.sp;
        self.regs.set_zn(self.regs.x);
    }

    /// Transfer x to accumulator
    fn txa(&mut self) {
        self.regs.a = self.regs.x;
        self.regs.set_zn(self.regs.a);
    }

    /// Transfer x to stack pointer
    fn txs(&mut self) {
        self.sp = self.regs.x;
    }

    /// Transfer y to accumulator
    fn tya(&mut self) {
        self.regs.a = self.regs.y;
        self.regs.set_zn(self.regs.a);
    }

    /// Illegal opcode: AHX
    fn ahx(&self) {
        panic!("Illegal opcode encountered: AXH");
    }

    /// Illegal opcode: ALR
    fn alr(&self) {
        panic!("Illegal opcode encountered: ALR");
    }

    /// Illegal opcode: ANC
    fn anc(&self) {
        panic!("Illegal opcode encountered: ANC");
    }

    /// Illegal opcode: ARR
    fn arr(&self) {
        panic!("Illegal opcode encountered: ARR");
    }

    /// Illegal opcode: AXS
    fn axs(&self) {
        panic!("Illegal opcode encountered: AXS");
    }

    /// DECs the contents of a memory location and then CMPs the result with the A register
    fn dcp(&mut self, addr: u16) {
        self.dec(addr);
        self.cmp(addr);
    }

    /// Illegal opcode: ISC
    fn isb(&mut self, addr: u16) {
        self.inc(addr);
        self.sbc(addr);
    }

    /// Illegal opcode: KIL
    fn kil(&self) {
        panic!("Illegal opcode encountered: KIL");
    }

    /// Illegal opcode: LAS
    fn las(&self) {
        panic!("Illegal opcode encountered: LAS");
    }

    /// Loads a byte of memory into the accumulator and x
    fn lax(&mut self, addr: u16) {
        let value = self.mem_read(addr);
        self.regs.x = value;
        self.regs.a = value;
        self.regs.set_zn(value);
    }

    /// Illegal opcode: RLA
    fn rla(&self) {
        panic!("Illegal opcode encountered: RLA");
    }

    /// Illegal opcode: RRA
    fn rra(&self) {
        panic!("Illegal opcode encountered: RRA");
    }

    /// Illegal opcode: SAX
    fn sax(&mut self, addr: u16) {
        self.mem_write(addr, self.regs.a & self.regs.x);
    }

    /// Illegal opcode: SHX
    fn shx(&self) {
        panic!("Illegal opcode encountered: SHX");
    }

    /// Illegal opcode: SHY
    fn shy(&self) {
        panic!("Illegal opcode encountered: SHY");
    }

    /// Illegal opcode: SLO
    fn slo(&self) {
        panic!("Illegal opcode encountered: SLO");
    }

    /// Illegal opcode: SRE
    fn sre(&self) {
        panic!("Illegal opcode encountered: SRE");
    }

    /// Illegal opcode: TAS
    fn tas(&self) {
        panic!("Illegal opcode encountered: TAS");
    }

    /// Illegal opcode: XAA
    fn xaa(&self) {
        panic!("Illegal opcode encountered: XAA");
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn verify_nestest() {
        let mut cpu = Cpu::new();
        let test_rom = include_bytes!("../../test-roms/nestest.nes").to_vec();
        let test_results = include_str!("../../test-roms/nestest-results.txt").lines();

        let rom_data = &test_rom[0x0010..0x4000];
        let rom_len = rom_data.len();

        cpu.memory[0x0800 .. (0x0800 + rom_data.len())].copy_from_slice(rom_data);
        cpu.memory[0xC000 .. (0xC000 + rom_data.len())].copy_from_slice(rom_data);

        cpu.regs.set_flags(0x24);
        cpu.sp = 0xFD;
        cpu.pc = 0xC000;

        for result in test_results {
            let actual = cpu.to_string();
            println!("{}", actual);
            assert_eq!(actual, result);
            cpu.step();
        }
    }

    impl fmt::Display for Cpu {
        fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
            let opcode = self.mem_read(self.pc);
            let byte_one = self.mem_read(self.pc + 1);
            let byte_two = self.mem_read(self.pc + 2);
            let mnemonic = &opcodes::INSTRUCTION_MNEMONIC[opcode as usize];
            let addressing = &opcodes::INSTRUCTION_MODES[opcode as usize];
            let address = self.get_next_operand_address(addressing);
            let operand = self.mem_read_u16(address);
            let address_bytes = u16::to_le_bytes(address);
            let operand_bytes = u16::to_le_bytes(operand);
            let rel_offset = self.mem_read(self.pc + 1);

            let bytes_fmt = match addressing {
                &AddressingMode::ZPG => format!("{:02X}", address_bytes[0]),
                &AddressingMode::ZPX => format!("{:02X}", byte_one),
                &AddressingMode::ZPY => format!("{:02X}", byte_one),
                &AddressingMode::ABS => format!("{:02X} {:02X}", address_bytes[0], address_bytes[1]),
                &AddressingMode::ABX => format!("{:02X} {:02X}", byte_one, byte_two),
                &AddressingMode::ABY => format!("{:02X} {:02X}", byte_one, byte_two),
                &AddressingMode::IND => format!("{:02X} {:02X}", byte_one, byte_two),
                &AddressingMode::IMP => format!(""),
                &AddressingMode::ACC => format!(""),
                &AddressingMode::IMM => format!("{:02X}", operand_bytes[0]),
                &AddressingMode::REL => format!("{:02X}", rel_offset),
                &AddressingMode::IDX => format!("{:02X}", byte_one),
                &AddressingMode::IDY => format!("{:02X}", byte_one),
                &AddressingMode::UNKNOWN => format!("{:02X} {:02X}", operand_bytes[0], operand_bytes[1])
            };

            let mnemonic_fmt = match addressing {
                &AddressingMode::ZPG => format!("{:?} ${:02X}", mnemonic, address_bytes[0]),
                &AddressingMode::ZPX => format!("{:?} ${:02X},X", mnemonic, byte_one),
                &AddressingMode::ZPY => format!("{:?} ${:02X},Y", mnemonic, byte_one),
                &AddressingMode::ABS => format!("{:?} ${:04X}", mnemonic, address),
                &AddressingMode::ABX => format!("{:?} ${:04X},X", mnemonic, u16::from_le_bytes([byte_one, byte_two])),
                &AddressingMode::ABY => format!("{:?} ${:04X},Y", mnemonic, u16::from_le_bytes([byte_one, byte_two])),
                &AddressingMode::IND => format!("{:?} (${:04X})", mnemonic, u16::from_le_bytes([byte_one, byte_two])),
                &AddressingMode::IMP => format!("{:?}", mnemonic),
                &AddressingMode::ACC => format!("{:?} A", mnemonic),
                &AddressingMode::IMM => format!("{:?} #${:02X}", mnemonic, operand_bytes[0]),
                &AddressingMode::REL => format!("{:?} ${:04X}", mnemonic, address),
                &AddressingMode::IDX => format!("{:?} (${:02X},X)", mnemonic, byte_one),
                &AddressingMode::IDY => format!("{:?} (${:02X}),Y", mnemonic, byte_one),
                &AddressingMode::UNKNOWN => format!("")
            };

            let mnemonic_illegal = match &opcodes::INSTRUCTION_ILLEGAL.contains(&opcode) {
                true => "*",
                false => " ",
            };

            write!(f, "{:04X}  {:02X} {: <5} {}{: <31} A:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X}",
                   self.pc,
                   opcode,
                   bytes_fmt,
                   mnemonic_illegal,
                   mnemonic_fmt,
                   self.regs.a,
                   self.regs.x,
                   self.regs.y,
                   self.regs.get_flags(),
                   self.sp
            )
        }
    }
}
