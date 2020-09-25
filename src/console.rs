use crate::cartridge::Cartridge;
use crate::cpu::CPU;

pub struct Console {
    cpu: CPU,
}

impl Console {
    pub fn new(cart: Cartridge) -> Self {
        Console {
            cpu: CPU::new(cart),
        }
    }
}
