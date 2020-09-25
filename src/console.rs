use crate::cpu;
use crate::cpu::opcodes::{AddressingMode, Mnemonic};
use crate::cpu::Cpu;
use crate::cartridge::Cartridge;

pub struct Console {
    cpu: Cpu,
}

impl Console {
    pub fn new(cart: Cartridge) -> Self {
        Console {
            cpu: Cpu::new(cart),
        }
    }
}
