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
            rom
        }
    }

    pub fn read_u8(&self, addr: u16) -> u8 {
        match addr {
            // Read from RAM (mirrored every 0x0800 bytes)
            0x0000..=0x07FF => {
                let ram_offset = (addr as usize) % self.ram.len();
                self.ram[ram_offset]
            }
            // Read from PRG-ROM (mirrored to fill all 32 KiB)
            0x0800..=0xFFF => {
                let rom_len = self.rom.prg_rom.len();
                let rom_offset = (addr as usize - 0x8000) % rom_len;
                self.rom.prg_rom[rom_offset]
            }
            _ => {
                unimplemented!("Read from ${:04X}", addr);
            }
        }
    }

    pub fn read_u16(&self, addr: u16) -> u16 {
        let low = self.read_u8(addr);
        let high = self.read_u8(addr.wrapping_add(1));

        (low as u16) | ((high as u16) << 8)
    }
}