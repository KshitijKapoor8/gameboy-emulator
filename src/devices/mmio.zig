const bus = @import("../bus.zig");
pub const PageReadFn = bus.PageReadFn;
pub const PageWriteFn = bus.PageWriteFn;

pub const NUM_MEMORY_MAP: u8 = 10;

pub const MMIO = struct {
    name: []const u8,
    read: PageReadFn,
    write: PageWriteFn,
    start_address: u16,
    end_address: u16,
};

// adjust to your real signatures
fn dummyRead(mem: []u8, offset: u8) u8 {
    _ = mem;
    _ = offset;
    return 0xFF;
}
fn dummyWrite(mem: []u8, offset: u8, value: u8) void {
    _ = mem;
    _ = offset;
    _ = value;
}

// this is now comptime-known
pub const MEMORY_MAP = [_]MMIO{
    // 0x0000–0x3FFF: fixed ROM
    .{
        .name = "ROM0",
        .read = dummyRead,
        .write = dummyWrite,
        .start_address = 0x0000,
        .end_address = 0x3FFF,
    },
    // 0x4000–0x7FFF: switchable ROM
    .{
        .name = "ROMX",
        .read = dummyRead,
        .write = dummyWrite,
        .start_address = 0x4000,
        .end_address = 0x7FFF,
    },
    // 0x8000–0x9FFF: VRAM
    .{
        .name = "VRAM",
        .read = dummyRead,
        .write = dummyWrite,
        .start_address = 0x8000,
        .end_address = 0x9FFF,
    },
    // 0xA000–0xBFFF: External/cart RAM
    .{
        .name = "EXT_RAM",
        .read = dummyRead,
        .write = dummyWrite,
        .start_address = 0xA000,
        .end_address = 0xBFFF,
    },
    // 0xC000–0xCFFF: Work RAM
    .{
        .name = "WRAM",
        .read = dummyRead,
        .write = dummyWrite,
        .start_address = 0xC000,
        .end_address = 0xCFFF,
    },
    // 0xD000–0xDFFF: Work RAM 2
    .{
        .name = "WRAM2",
        .read = dummyRead,
        .write = dummyWrite,
        .start_address = 0xD000,
        .end_address = 0xDFFF,
    },
    // 0xE000–0xFDFF: Echo RAM
    .{
        .name = "ECHO",
        .read = dummyRead,
        .write = dummyWrite,
        .start_address = 0xE000,
        .end_address = 0xFDFF,
    },
    // 0xFE00–0xFEFF: OAM
    .{
        .name = "OAM",
        .read = dummyRead,
        .write = dummyWrite,
        .start_address = 0xFE00,
        .end_address = 0xFEFF,
    },
    // 0xFF00–0xFFFF: I/O + HRAM
    .{
        .name = "IO_HRAM",
        .read = dummyRead,
        .write = dummyWrite,
        .start_address = 0xFF00,
        .end_address = 0xFFFF,
    },
    // filler
    .{
        .name = "UNUSED",
        .read = dummyRead,
        .write = dummyWrite,
        .start_address = 0x0000,
        .end_address = 0x0000,
    },
};
