use crate::cartridge::{Cartridge, NesRom};
use crate::cartridge::Cartridge::{iNES, NES2};

macro_rules! mem_range_ram {
    () => {
        0x0000..=0x1FFF
    };
}
macro_rules! mem_range_ppu {
    () => {
        0x2000..=0x3FFF
    };
}
macro_rules! mem_range_rom {
    () => {
        0x8000..=0xFFFF
    };
}

pub trait Memory {
    fn read(&self, addr: u16) -> u8;

    fn write(&mut self, addr: u16, data: u8);

    fn read_u16(&self, pos: u16) -> u16;

    fn write_u16(&mut self, pos: u16, data: u16);
}

pub struct Bus {
    ram: [u8; 2048],
    rom: NesRom,
}

impl Bus {
    pub fn new(cart: Cartridge) -> Self {
        let rom = match cart {
            iNES(r) => r,
            NES2(r) => r,
        };

        Bus {
            ram: [0; 2048],
            rom,
        }
    }
}

impl Bus {
    fn read_from_prg(&self, addr: u16) -> u8 {
        let prg_addr = if self.rom.prg.len() == 0x4000 && addr >= 0x4000 {
            // Mirror the address if needed
            (addr - 0x8000) % 0x4000
        } else {
            // Get the address in the actual PRG ROM
            (addr - 0x8000)
        };

        self.rom.prg[prg_addr as usize]
    }
}

impl Memory for Bus {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            mem_range_ram!() => {
                // Due to mirroring in the RAM, only 11 of the 13 bits of the address are used
                // https://wiki.nesdev.com/w/index.php/Mirroring#Memory_Mirroring
                let addr_mirrored = addr & 0b0111_1111_1111;
                self.ram[addr_mirrored as usize]
            }
            mem_range_ppu!() => todo!("PPU needs to be implemented!"),
            mem_range_rom!() => self.read_from_prg(addr),
            _ => {
                println!("Ignoring memory read at 0x{:04X}", addr);
                0
            }
        }
    }

    fn write(&mut self, addr: u16, data: u8) {
        match addr {
            mem_range_ram!() => {
                // Due to mirroring in the RAM, only 11 of the 13 bits of the address are used
                // https://wiki.nesdev.com/w/index.php/Mirroring#Memory_Mirroring
                let addr_mirrored = addr & 0b0111_1111_1111;
                self.ram[addr_mirrored as usize] = data;
            }
            mem_range_ppu!() => todo!("PPU is not supported yet"),
            mem_range_rom!() => panic!("Attempt to write to Cartridge ROM space"),
            _ => {
                println!("Ignoring memory write at 0x{:04X}", addr);
            }
        }
    }

    fn read_u16(&self, addr: u16) -> u16 {
        let lo = self.read(addr);
        let hi = self.read(addr + 1);
        u16::from_le_bytes([lo, hi])
    }

    fn write_u16(&mut self, addr: u16, data: u16) {
        let [one, two] = data.to_le_bytes();
        self.write(addr, one);
        self.write(addr + 1, two);
    }
}
