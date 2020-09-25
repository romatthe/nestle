use crate::cartridge::Cartridge;
use crate::cpu::opcodes::AddressingMode;
use crate::cpu::{opcodes, Cpu};
use std::fmt;
use std::fmt::Formatter;

#[test]
fn verify_nestest() {
    let test_rom = include_bytes!("../../../test-roms/nestest.nes").to_vec();
    let test_results = include_str!("../../../test-roms/nestest-results.txt").lines();
    let cart = Cartridge::from_raw(&test_rom).unwrap();

    let mut cpu = Cpu::new(cart);

    // The NESTEST reset vector points to 0xC004, but it has an automated run mode if you point
    // the program counter to 0xC000, which is what we want to use in this test
    cpu.reset();
    cpu.pc = 0xC000;

    for result in test_results {
        assert_eq!(cpu.to_string(), result);
        cpu.step();
    }

    // Read bytes from 0x02 and 0x03 from memory to get an error code, 0x0000 indicates success
    let result = cpu.mem_read_u16(0x0002);
    assert_eq!(
        result,
        0x0000,
        "NESTEST failed: {}",
        super::NESTEST_ERRORS[&result]
    );
}

impl fmt::Display for Cpu {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let opcode = self.mem_read(self.pc);
        let mnemonic = &opcodes::INSTRUCTION_MNEMONIC[opcode as usize];
        let addressing = &opcodes::INSTRUCTION_MODES[opcode as usize];
        let address = self.get_next_operand_address(addressing);
        let operand = self.mem_read_u16(address);
        let [byte_one, byte_two] = u16::to_le_bytes(self.mem_read_u16(self.pc + 1));
        let [addr_one, addr_two] = u16::to_le_bytes(address);
        let [oper_one, oper_two] = u16::to_le_bytes(operand);
        let rel_offset = self.mem_read(self.pc + 1);

        let bytes_fmt = match addressing {
            &AddressingMode::ZPG => format!("{:02X}", addr_one),
            &AddressingMode::ZPX => format!("{:02X}", byte_one),
            &AddressingMode::ZPY => format!("{:02X}", byte_one),
            &AddressingMode::ABS => format!("{:02X} {:02X}", addr_one, addr_two),
            &AddressingMode::ABX => format!("{:02X} {:02X}", byte_one, byte_two),
            &AddressingMode::ABY => format!("{:02X} {:02X}", byte_one, byte_two),
            &AddressingMode::IND => format!("{:02X} {:02X}", byte_one, byte_two),
            &AddressingMode::IMP => format!(""),
            &AddressingMode::ACC => format!(""),
            &AddressingMode::IMM => format!("{:02X}", oper_one),
            &AddressingMode::REL => format!("{:02X}", rel_offset),
            &AddressingMode::IDX => format!("{:02X}", byte_one),
            &AddressingMode::IDY => format!("{:02X}", byte_one),
            &AddressingMode::UNKNOWN => format!("{:02X} {:02X}", oper_one, oper_two),
        };

        let mnemonic_fmt = match addressing {
            &AddressingMode::ZPG => format!("{:?} ${:02X}", mnemonic, addr_one),
            &AddressingMode::ZPX => format!("{:?} ${:02X},X", mnemonic, byte_one),
            &AddressingMode::ZPY => format!("{:?} ${:02X},Y", mnemonic, byte_one),
            &AddressingMode::ABS => format!("{:?} ${:04X}", mnemonic, address),
            &AddressingMode::ABX => format!(
                "{:?} ${:04X},X",
                mnemonic,
                u16::from_le_bytes([byte_one, byte_two])
            ),
            &AddressingMode::ABY => format!(
                "{:?} ${:04X},Y",
                mnemonic,
                u16::from_le_bytes([byte_one, byte_two])
            ),
            &AddressingMode::IND => format!(
                "{:?} (${:04X})",
                mnemonic,
                u16::from_le_bytes([byte_one, byte_two])
            ),
            &AddressingMode::IMP => format!("{:?}", mnemonic),
            &AddressingMode::ACC => format!("{:?} A", mnemonic),
            &AddressingMode::IMM => format!("{:?} #${:02X}", mnemonic, oper_one),
            &AddressingMode::REL => format!("{:?} ${:04X}", mnemonic, address),
            &AddressingMode::IDX => format!("{:?} (${:02X},X)", mnemonic, byte_one),
            &AddressingMode::IDY => format!("{:?} (${:02X}),Y", mnemonic, byte_one),
            &AddressingMode::UNKNOWN => format!(""),
        };

        let mnemonic_illegal = match &opcodes::INSTRUCTION_ILLEGAL.contains(&opcode) {
            true => "*",
            false => " ",
        };

        write!(
            f,
            "{:04X}  {:02X} {: <5} {}{: <31} A:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X}",
            self.pc,
            opcode,
            bytes_fmt,
            mnemonic_illegal,
            mnemonic_fmt,
            self.regs.a,
            self.regs.x,
            self.regs.y,
            self.regs.get_flags(),
            self.sp
        )
    }
}
