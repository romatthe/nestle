use crate::cpu;
use crate::cpu::opcodes::{AddressingMode, Mnemonic};
use crate::cpu::Cpu;

pub struct Rom {
    prg_rom: Vec<u8>,
}

pub struct Console {
    cpu: Cpu,
    ram: [u8; 2048],
    rom: Rom,
}

impl Console {
    pub fn from_rom(rom: Rom) -> Self {
        Console {
            cpu: Cpu::new(),
            ram: [0; 2048],
            rom,
        }
    }
}
