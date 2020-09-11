use crate::cpu::opcodes::{Mnemonic, AddressingMode};
use crate::cpu::bus::Bus;
use log::warn;

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

    /// Set Zero flag if A = 0 and Negative flag if bit 7 of A is set
    fn set_zn(&mut self, result: u8) {
        self.set_z(result);
        self.set_n(result);
    }

    /// Set Zero flag if A = 0
    fn set_z(&mut self, result: u8) {
        if result == 0 {
            self.z = 1;
        } else {
            self.z = 0;
        }
    }

    /// Set Negative flag if bit 7 of A is set
    fn set_n(&mut self, result: u8) {
        if result & 0b1000_0000 != 0 {
            self.n = 1;
        } else {
            self.n = 0;
        }
    }
}

pub struct Cpu {
    /// Program counter
    pc: u16,
    /// Registers
    regs: Registers,
    /// RAM
    memory: [u8; 0xFFFF]
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            pc: 0,
            regs: Registers::new(),
            memory: [0x0; 0xFFFF]
        }
    }

    pub fn run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.exec();
    }

    fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    fn mem_read_u16(&mut self, addr: u16) -> u16 {
        u16::from_le_bytes([self.mem_read(addr), self.mem_read(addr + 1)])
    }

    fn mem_write_u16(&mut self, addr: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xFF) as u8;
        self.mem_write(addr, lo);
        self.mem_write(addr + 1, hi);
    }

    pub fn reset(&mut self) {
        // Reset the registers
        self.regs.a = 0;
        self.regs.x = 0;
        self.regs.set_flags(0);

        // Set the program counter to the start of the program as stored in 0xFFFC during load
        self.pc = self.mem_read_u16(0xFFFC);
    }

    fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);

        // Store the reference to the start of the code in 0xFFFC
        self.mem_write_u16(0xFFFC, 0x8000);
    }

    pub fn step(&mut self) {

    }

    /// Execute an instruction in the current CPU state
    fn exec(&mut self) {
        loop {
            // Fetch the next instruction
            let opcode = self.mem_read(self.pc);
            self.pc += 1;

            match opcode {
                0x00 => {
                    // Exit
                    return;
                }
                0xA9 => {
                    let param = self.mem_read(self.pc);
                    self.pc += 1;
                    self.lda(param);
                }
                0xAA => {
                    self.tax();
                }
                0xE8 => {
                    self.inx();
                }
                _ => todo!("")
            }
        }
    }

    fn inx(&mut self) {
        self.regs.x = self.regs.x.wrapping_add(1);
        self.regs.set_zn(self.regs.x);
    }

    fn lda(&mut self, value: u8) {
        self.regs.a = value;
        self.regs.set_zn(self.regs.a);
    }

    fn tax(&mut self) {
        self.regs.x = self.regs.a;
        self.regs.set_zn(self.regs.x);
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_lda_immediate_load_data() {
        let mut cpu = Cpu::new();
        cpu.run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.regs.a, 0x05);
        assert_eq!(cpu.regs.z, 0);
        assert_eq!(cpu.regs.n, 0);
    }

    #[test]
    fn test_tax_transfer_a_to_x() {
        let mut cpu = Cpu::new();
        cpu.load(vec![0xaa, 0x00]);
        cpu.reset();
        cpu.regs.a = 10;
        cpu.exec();

        assert_eq!(cpu.regs.x, 10)
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = Cpu::new();
        cpu.run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.regs.x, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = Cpu::new();
        cpu.load(vec![0xe8, 0xe8, 0x00]);
        cpu.reset();
        cpu.regs.x = 0xff;
        cpu.exec();

        assert_eq!(cpu.regs.x, 1)
    }
}
