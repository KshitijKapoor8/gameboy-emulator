const std = @import("std");

pub const Cartridge = struct {
    rom: []const u8,
    
    pub fn init(rom: []const u8) Cartridge {
        return .{.rom = rom};
    }

    pub fn read(self: *const Cartridge, addr: u16) u8 {
        // cartridge address space is 0x0000 - 0x7FFF for now
        if (addr < 0x8000 and addr < self.rom.len)
        {
            return self.rom[addr];
        }
        return 0xFF;
    }

    // ignoring writes for now
    pub fn write(self: *Cartridge, addr: u16, value: u8) void {
        _ = self;
        _ = addr;
        _ = value;
    }
};