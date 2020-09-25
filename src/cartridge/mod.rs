use crate::cartridge::Cartridge::iNES;

/// The magic number defining a file as a ROM in the iNES format.
/// ("NES" followed by MS-DOS end-of-file).
const NES_MAGIC_NUMBER: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];
/// Size of a single memory page inside the PRG ROM
const PRG_ROM_PAGE_SIZE: usize = 0x4000;
/// Size of a single memory page inside the CHR ROM
const CHR_ROM_PAGE_SIZE: usize = 0x2000;

/// A cartridge and its corresponding ROM data.
pub enum Cartridge {
    /// A cartridge with ROM data from a file in the iNES format.
    iNES(ROM),
    /// A cartridge with ROM data from a file in the NES2.0 format.
    NES2(ROM),
}

#[derive(Debug)]
pub enum Mirroring {
    Vertical,
    Horizontal,
    FourScreen,
}

pub struct ROM {
    /// Data of the ROM memory connected to the CPU.
    pub prg: Vec<u8>,
    /// Data of the ROM memory connected to the PPU.
    pub chr: Vec<u8>,
    /// Id of the mapper used in this ROM (mapper support additional capabilities inside the cart.
    /// by including extra hardware)
    pub mapper: u8,
    /// What type of nametable mirroring this ROM supports.
    pub mirroring: Mirroring,
}

impl Cartridge {
    /// Parses an NES ROM from a byte string.
    pub fn from_raw(raw: &Vec<u8>) -> Result<Cartridge, String> {
        // Check if this is actually an iNES file
        if &raw[0..4] != NES_MAGIC_NUMBER {
            return Err("File is not in iNES file format".to_string());
        }

        // Extract the id of the mapper this cartridge supports
        let mapper = (raw[7] & 0b1111_0000) | (raw[6] >> 4);

        // Extract the version of the NES file format
        let ines_ver = (raw[7] >> 2) & 0b11;
        if ines_ver != 0 {
            return Err("NES2.0 format is not supported".to_string());
        }

        // Extract the mirroring mode
        let four_screen = raw[6] & 0b1000 != 0;
        let vertical = raw[6] & 0b1 != 0;
        let mirroring = match (four_screen, vertical) {
            (true, _) => Mirroring::FourScreen,
            (false, true) => Mirroring::Vertical,
            (false, false) => Mirroring::Horizontal,
        };

        // Determine the total size of each type of ROM memory
        let prg_rom_size = raw[4] as usize * PRG_ROM_PAGE_SIZE;
        let chr_rom_size = raw[5] as usize * CHR_ROM_PAGE_SIZE;

        let skip_trainer = raw[6] & 0b100 != 0;

        let prg_rom_start = 16 + if skip_trainer { 512 } else { 0 };
        let chr_rom_start = prg_rom_start + prg_rom_size;

        let rom = ROM {
            prg: raw[prg_rom_start..(prg_rom_start + prg_rom_size)].to_vec(),
            chr: raw[chr_rom_start..(chr_rom_start + chr_rom_size)].to_vec(),
            mapper,
            mirroring,
        };

        Ok(iNES(rom))
    }
}
