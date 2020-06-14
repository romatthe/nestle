pub struct Bus {
    ram: [u8; 2048],
}

impl Bus {
    pub fn new() -> Self {
        Bus {
            ram: [0; 2048],
        }
    }

    pub fn read_u8(&self, addr: u16) -> u8 {
        match addr {
            // Read from RAM (mirrored every 0x0800 bytes)
            0..=0x1FFF => self.ram[addr as usize % 0x0800],
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

    pub fn write_u8(&mut self, addr: u16, value: u8) {
        match addr {
            0..=0x1FFF => self.ram[addr as usize % 0x0800] = value,
            _ => {
                unimplemented!("Read from ${:04X}", addr);
            }
        }
    }
}