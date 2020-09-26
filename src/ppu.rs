use crate::cartridge::Mirroring;
use crate::ppu::AddressState::{Low, High};
use std::ops::Not;

/// The NES 2C02 Picture Processing Unit
pub struct PPU {
    /// Visual data of the game retrieved from the cartridge ROM
    pub chr_rom: Vec<u8>,
    /// Mirroring configuration for this current game
    pub mirroring: Mirroring,
    /// Internal memory holding palette tables used by a screen
    pub palette_table: [u8; 32],
    /// Internal memory holding background data
    pub vram: [u8; 2048],
    /// Internal memory holding the state of sprites
    pub oam_data: [u8; 256],
    /// Used by the CPU to request CHR ROM data reads by the PPU, known as `PPUADDR`
    pub address_reg: AddressRegister,
}

impl PPU {
    pub fn new(chr_rom: Vec<u8>, mirroring: Mirroring) -> Self {
        PPU {
            chr_rom,
            mirroring,
            palette_table: [0; 32],
            vram: [0; 2048],
            oam_data: [0; 256],
            address_reg: AddressRegister::new(),
        }
    }

    fn write_to_address_reg(&mut self, value: u8) {
        self.address_reg.update(value);
    }
}

#[derive(Clone, Copy)]
enum AddressState {
    High,
    Low,
}

impl Not for AddressState {
    type Output = AddressState;

    fn not(self) -> Self::Output {
        match self {
            Low => High,
            High => Low
        }
    }
}

/// This register is used by the CPU to request CHR ROM data reads by the PPU.
pub struct AddressRegister {
    /// The two bytes written to address 0x2006 by the CPU. It uses Big Endian notation.
    value: (u8, u8),
    /// Tracks if the high or low byte is to be written to $2006 next
    state: AddressState,
}

impl AddressRegister {
    pub fn new() -> Self {
        AddressRegister {
            value: (0, 0),
            state: Low
        }
    }

    pub fn get(&self) -> u16 {
        u16::from_be_bytes([self.value.0, self.value.1])
    }

    fn set(&mut self, data: u16) {
        let [hi, lo] = data.to_be_bytes();
        self.value.0 = hi;
        self.value.1 = lo;
    }

    pub fn update(&mut self, data: u8) {
        match &self.state {
            Low => self.value.0 = data,
            High => self.value.1 = data
        }

        // The PPU registers are mirrored in every 8 bytes from $2008 through $3FFF
        if self.get() > 0x3FFF { //
            // Mirror down address above 0x3FFF
            self.set(self.get() & 0b11111111111111);
        }

        // Flip the state
        self.state = !self.state;
    }

    pub fn increment(&mut self, add: u8) {
        let lo = self.value.1;
        self.value.1 = self.value.1.wrapping_add(add);

        if lo > self.value.1 {
            self.value.0 = self.value.0.wrapping_add(1);
        }

        // The PPU registers are mirrored in every 8 bytes from $2008 through $3FFF
        if self.get() > 0x3FFF {
            // Mirror down address above 0x3FFF
            self.set(self.get() & 0b11111111111111);
        }
    }

    pub fn reset(&mut self) {
        self.state = Low;
    }
}