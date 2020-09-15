use crate::cpu::opcodes::{Mnemonic, AddressingMode, INSTRUCTION_SIZES};
use crate::cpu::bus::Bus;
use log::warn;
use std::collections::HashMap;

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
    running: bool
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            pc: 0,
            sp: STACK_RESET,
            regs: Registers::new(),
            memory: [0x0; 0xFFFF],
            running: true
        }
    }

    pub fn run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();

        while self.running {
            let opcode = self.mem_read(self.pc);
            let mode = &opcodes::INSTRUCTION_MODES[opcode as usize];
            let mnemonic = &opcodes::INSTRUCTION_MNEMONIC[opcode as usize];

            self.exec(opcode, mnemonic, mode);
        }
    }

    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut Cpu),
    {
        while self.running {
            let opcode = self.mem_read(self.pc);
            let mode = &opcodes::INSTRUCTION_MODES[opcode as usize];
            let mnemonic = &opcodes::INSTRUCTION_MNEMONIC[opcode as usize];

            // Execute the callback
            callback(self);

            self.exec(opcode, mnemonic, mode);
        }
    }

    pub fn reset(&mut self) {
        // Reset the registers
        self.regs.a = 0;
        self.regs.x = 0;
        self.regs.set_flags(0);
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

    pub fn mem_read_u16(&mut self, addr: u16) -> u16 {
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
        let address = STACK_OFFSET + self.sp as u16;
        self.mem_write(address, value);
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
        self.mem_read(STACK_OFFSET + (self.sp as u16))
    }

    /// Pop two bytes off the stack
    fn pop_u16(&mut self) -> u16 {
        let lo = self.pop_u8() as u16;
        let hi = self.pop_u8() as u16;
        hi << 8 | lo
    }

    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);

        // Store the reference to the start of the code in 0xFFFC
        self.mem_write_u16(0xFFFC, 0x8000);
    }

    /// Get the address of the next operand based on the addressing mode
    fn get_next_operand_address(&mut self, mode: &AddressingMode) -> u16 {
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
                let base = self.mem_read(pc_next).wrapping_add(self.regs.x);
                let lo = self.mem_read(base as u16);
                let hi = self.mem_read(base.wrapping_add(1) as u16);

                u16::from_le_bytes([lo, hi])
            },
            // Indirect addressing
            AddressingMode::IND => {
                let base = self.mem_read_u16(pc_next);

                self.mem_read_u16(base)
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
                    pc_next.wrapping_add(1 + offset)
                } else {
                    pc_next.wrapping_add(1 + offset - 0x100)
                }
            }
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
        let operand = self.get_next_operand_address(mode);
        let size = &opcodes::INSTRUCTION_SIZES[opcode as usize];

        match mnemonic {
            Mnemonic::ADC => self.adc(mode),
            Mnemonic::AND => self.and(mode),
            Mnemonic::ASL => self.asl(mode),
            Mnemonic::BCC => self.bcc(mode),
            Mnemonic::BCS => self.bcs(mode),
            Mnemonic::BEQ => self.beq(mode),
            Mnemonic::BIT => self.bit(mode),
            Mnemonic::BMI => self.bmi(mode),
            Mnemonic::BNE => self.bne(mode),
            Mnemonic::BPL => self.bpl(mode),
            Mnemonic::BRK => self.brk(),
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
            Mnemonic::PHP => self.php(),
            Mnemonic::PLA => self.pla(),
            Mnemonic::PLP => self.plp(),
            Mnemonic::ROL => self.rol(mode),
            Mnemonic::ROR => self.ror(mode),
            Mnemonic::RTI => self.rti(),
            Mnemonic::RTS => self.rts(),
            Mnemonic::SBC => self.sbc(mode),
            Mnemonic::SEC => self.sec(),
            Mnemonic::SED => self.sed(),
            Mnemonic::SEI => self.sei(mode),
            Mnemonic::STA => self.sta(mode),
            Mnemonic::STX => self.stx(mode),
            Mnemonic::STY => self.sty(mode),
            Mnemonic::TAX => self.tax(),
            Mnemonic::TAY => self.tay(),
            Mnemonic::TSX => self.tsx(),
            Mnemonic::TXA => self.txa(),
            Mnemonic::TXS => self.txs(),
            Mnemonic::TYA => self.tya(),

            // Illegal opcodes
            Mnemonic::AHX => self.ahx(),
            Mnemonic::ALR => self.alr(),
            Mnemonic::ANC => self.anc(),
            Mnemonic::ARR => self.arr(),
            Mnemonic::AXS => self.axs(),
            Mnemonic::DCP => self.dcp(),
            Mnemonic::ISC => self.isc(),
            Mnemonic::KIL => self.kil(),
            Mnemonic::LAS => self.las(),
            Mnemonic::LAX => self.lax(),
            Mnemonic::RLA => self.rla(),
            Mnemonic::RRA => self.rra(),
            Mnemonic::SAX => self.sax(),
            Mnemonic::SHX => self.shx(),
            Mnemonic::SHY => self.shy(),
            Mnemonic::SLO => self.slo(),
            Mnemonic::SRE => self.sre(),
            Mnemonic::TAS => self.tas(),
            Mnemonic::XAA => self.xaa(),

            // Unknown
            Mnemonic::UNKNOWN => panic!("Unknown opcode encountered: {:#X?}", opcode),
            _ => panic!("Unknown opcode encountered: {:#X?}", opcode),
        }

        // Increase the program counter by the size of the instruction
        self.pc = self.pc.wrapping_add(*size as u16);
    }

    /// Branch to the specified address if condition is true
    fn branch(&mut self, condition: bool) {
        if condition {
            let jump: i8 = self.mem_read(self.pc) as i8;
            let jump_addr = self
                .pc
                .wrapping_add(1)
                .wrapping_add(jump as u16);

            self.pc = jump_addr;
        }
    }

    /// Compare values and set the zero and carry flags as appropriate
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
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address);

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
        if (operand ^ result) & (result ^ self.regs.a) & 0x80 != 0 {
            self.regs.v = 1;
        } else {
            self.regs.v = 0;
        }
    }

    /// Logical AND
    fn and(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address);

        self.regs.a = self.regs.a & operand;
        self.regs.set_zn(self.regs.a);
    }

    /// Arithmetic Shift Left
    fn asl(&mut self, mode: &AddressingMode) {
        if *mode == AddressingMode::ACC {
            // Set the carry flag accordingly
            self.regs.c = (self.regs.a >> 7) & 1;

            self.regs.a <<= 1;
            self.regs.set_zn(self.regs.a);
        } else {
            let address = self.get_next_operand_address(mode);
            let operand = self.mem_read(address);
            let value = operand << 1;

            self.regs.c = (operand >> 7) & 1;
            self.mem_write(address, value);
            self.regs.set_zn(value);
        }
    }

    /// Branch if carry clear
    fn bcc(&mut self, mode: &AddressingMode) {
        self.branch(self.regs.c == 0);
    }

    /// Branch if carry set
    fn bcs(&mut self, mode: &AddressingMode) {
        self.branch(self.regs.c == 1);
    }

    /// Branch if equal
    fn beq(&mut self, mode: &AddressingMode) {
        self.branch(self.regs.z == 0);
    }

    /// Bit test
    fn bit(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address);

        self.regs.v = (operand >> 6) & 1;
        self.regs.set_z(operand & self.regs.a);
        self.regs.set_n(operand);
    }

    /// Branch if minus
    fn bmi(&mut self, mode: &AddressingMode) {
        self.branch(self.regs.n == 1)
    }

    /// Branch if not equal
    fn bne(&mut self, mode: &AddressingMode) {
        self.branch(self.regs.z == 0)
    }

    /// Branch if positive
    fn bpl(&mut self, mode: &AddressingMode) {
        self.branch(self.regs.n == 0)
    }

    /// Force interrupt
    fn brk(&mut self) {
        // TODO: Figure out the correct way to handle interrupts
        // self.running = false;
    }

    /// Branch if overflow clear
    fn bvc(&mut self, mode: &AddressingMode) {
        self.branch(self.regs.v == 0)
    }

    /// Branch if overflow set
    fn bvs(&mut self, mode: &AddressingMode) {
        self.branch(self.regs.v == 1)
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
    fn cmp(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address);

        self.compare(self.regs.a, operand);
    }

    /// Compare X Register
    fn cpx(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address);

        self.compare(self.regs.x, operand);
    }

    /// Compare X Register
    fn cpy(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address);

        self.compare(self.regs.y, operand);
    }

    /// Decrement memory
    fn dec(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address).wrapping_sub(1);

        self.mem_write(address, operand);
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
    fn eor(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address).wrapping_sub(1);

        self.regs.a = self.regs.a ^ operand;
        self.regs.set_zn(self.regs.a);
    }

    /// Increment memory
    fn inc(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address).wrapping_add(1);

        self.mem_write(address, operand);
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
    fn jmp(&mut self, mode: &AddressingMode) {
        if *mode == AddressingMode::ABS {
            self.pc = self.mem_read_u16(self.pc);
        } else if *mode == AddressingMode::IND {
            // An original 6502 has does not correctly fetch the target address if the indirect vector
            // falls on a page boundary (e.g. $xxFF where xx is any value from $00 to $FF). In this case
            // fetches the LSB from $xxFF as expected but takes the MSB from $xx00.
            let address = self.mem_read_u16(self.pc);
            let indirect_ref = if address & 0x00FF == 0x00FF {
                let lo = self.mem_read(address);
                let hi = self.mem_read(address & 0xFF00);
                (hi as u16) << 8 | (lo as u16)
            } else {
                self.mem_read_u16(address)
            };

            self.pc = indirect_ref;
        }
    }

    /// Jump to subroutine
    fn jsr(&mut self, mode: &AddressingMode) {
        self.push_u16(self.pc.wrapping_sub(1));
        self.pc = self.get_next_operand_address(mode);
    }

    /// Load accumulator
    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_next_operand_address(&mode);
        let operand = self.mem_read(addr);

        self.regs.a = operand;
        self.regs.set_zn(self.regs.a);
    }

    /// Load x register
    fn ldx(&mut self, mode: &AddressingMode) {
        let addr = self.get_next_operand_address(&mode);
        let operand = self.mem_read(addr);

        self.regs.x = operand;
        self.regs.set_zn(self.regs.x);
    }

    /// Load y register
    fn ldy(&mut self, mode: &AddressingMode) {
        let addr = self.get_next_operand_address(&mode);
        let operand = self.mem_read(addr);

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
            let address = self.get_next_operand_address(mode);
            let mut operand = self.mem_read(address);

            self.regs.c = operand & 1;
            operand >>= 1;

            self.mem_write(address, operand);
            self.regs.set_zn(operand);
        }
    }

    /// No operation
    fn nop(&self) {

    }

    /// Logical inclusive or
    fn ora(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address);

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
    fn rol(&mut self, mode: &AddressingMode) {
        let c = self.regs.c;

        if *mode == AddressingMode::ACC {
            self.regs.c = (self.regs.a >> 7) & 1;
            self.regs.a = (self.regs.a << 1) | c;
            self.regs.set_zn(self.regs.a);
        } else {
            let address = self.get_next_operand_address(mode);
            let mut operand = self.mem_read(address);

            self.regs.c = (operand >> 7) & 1;
            self.mem_write(address, (operand << 1) | c);
            self.regs.set_zn(operand);
        }
    }

    /// Rotate right
    fn ror(&mut self, mode: &AddressingMode) {
        let c = self.regs.c;

        if *mode == AddressingMode::ACC {
            self.regs.c = self.regs.a & 1;
            self.regs.a = (self.regs.a >> 1) | (c << 7);
            self.regs.set_zn(self.regs.a);
        } else {
            let address = self.get_next_operand_address(mode);
            let mut operand = self.mem_read(address);

            self.regs.c = operand & 1;
            self.mem_write(address, (operand >> 1) | (c << 7));
            self.regs.set_zn(operand);
        }
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
        self.pc = self.pop_u16().wrapping_add(1);
    }

    /// Subtract with Carry
    fn sbc(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        let operand = self.mem_read(address);
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
        if (data ^ result) & (result ^ self.regs.a) & 0x80 != 0 {
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
    fn sei(&mut self, mode: &AddressingMode) {
        self.regs.i = 1;
    }

    /// Store accumulator
    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_next_operand_address(mode);
        self.mem_write(addr, self.regs.a);
    }

    /// Store the x register
    fn stx(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        self.mem_write(address, self.regs.x);
    }

    /// Store the y register
    fn sty(&mut self, mode: &AddressingMode) {
        let address = self.get_next_operand_address(mode);
        self.mem_write(address, self.regs.y);
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

    /// Illegal opcode: DCP
    fn dcp(&self) {
        panic!("Illegal opcode encountered: DCP");
    }

    /// Illegal opcode: ISC
    fn isc(&self) {
        panic!("Illegal opcode encountered: ISC");
    }

    /// Illegal opcode: KIL
    fn kil(&self) {
        panic!("Illegal opcode encountered: KIL");
    }

    /// Illegal opcode: LAS
    fn las(&self) {
        panic!("Illegal opcode encountered: LAS");
    }

    /// Illegal opcode: LAX
    fn lax(&self) {
        panic!("Illegal opcode encountered: LAX");
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
    fn sax(&self) {
        panic!("Illegal opcode encountered: SAX");
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
    fn test_inx() {
        let mut cpu = Cpu::new();
        cpu.run(vec![0xe8, 0x00]);

        assert_eq!(cpu.regs.x, 1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = Cpu::new();
        cpu.run(vec![0xa9, 0xff, 0xaa, 0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.regs.x, 1)
    }

    #[test]
    fn test_lda_immediate() {
        let mut cpu = Cpu::new();
        cpu.run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.regs.a, 5);
        assert_eq!(cpu.regs.c, 0);
        assert_eq!(cpu.regs.n, 0);
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = Cpu::new();
        cpu.mem_write(0x10, 0x55);

        cpu.run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.regs.a, 0x55);
    }

    #[test]
    fn test_lda_zero_flag() {
        let mut cpu = Cpu::new();
        cpu.run(vec![0xa9, 0x00, 0x00]);
        assert_eq!(cpu.regs.z, 1);
    }

    #[test]
    fn test_sta_absolute() {
        let mut cpu = Cpu::new();

        // Write 0x55 to location 0x10
        cpu.mem_write(0x10, 0x55);

        // Load a from location 0x10 the store a in location 0x20
        cpu.run(vec![0xa5, 0x10, 0x8d, 0x20, 0x00, 0x00]);

        assert_eq!(cpu.mem_read(0x20), 0x55)
    }

    #[test]
    fn test_tax() {
        let mut cpu = Cpu::new();
        cpu.run(vec![0xa9, 0x0A,0xaa, 0x00]);

        assert_eq!(cpu.regs.x, 10)
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = Cpu::new();
        cpu.run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.regs.x, 0xc1)
    }
}
