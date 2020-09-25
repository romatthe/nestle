use crate::cartridge::Mirroring;

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
}

impl PPU {
    pub fn new(chr_rom: Vec<u8>, mirroring: Mirroring) -> Self {
        PPU {
            chr_rom,
            mirroring,
            palette_table: [0; 32],
            vram: [0; 2048],
            oam_data: [0; 256],
        }
    }
}