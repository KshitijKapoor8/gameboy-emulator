const bus = @import("../bus.zig");
const cart_mmio = @import("cart_boot_mmio.zig");

pub const PageReadFn = bus.PageReadFn;
pub const PageWriteFn = bus.PageWriteFn;

pub const NUM_MEMORY_MAP: u8 = 9;

const std = @import("std");

// simple serial log buffer for tests
pub var serial_log: [4096]u8 = undefined;
pub var serial_len: usize = 0;

pub fn serialReset() void {
    serial_len = 0;
}

pub fn serialSlice() []const u8 {
    return serial_log[0..serial_len];
}

fn serialAppend(byte: u8) void {
    if (serial_len < serial_log.len) {
        serial_log[serial_len] = byte;
        serial_len += 1;
    }
}

fn ioRead(mem: []u8, offset: u8) u8 {
    return mem[offset];
}

fn ioWrite(mem: []u8, offset: u8, value: u8) void {
    mem[offset] = value;

    // serial: SB = 0xFF01, SC = 0xFF02
    // Inside this page, offset 0x01 -> SB, 0x02 -> SC
    if (offset == 0x01) {
        // SB: byte to send; just stored above
        return;
    }

    if (offset == 0x02) {
        // SC: when 0x81 is written, transmit SB
        if ((value & 0x81) == 0x81) {
            const sb = mem[0x01]; // byte in SB
            serialAppend(sb);
            // also print to stderr for manual debugging if you want:
            // std.debug.print("{c}", .{sb});
        }
    }
}

pub const MMIO = struct {
    name: []const u8,
    read: PageReadFn,
    write: PageWriteFn,
    start_address: u16,
    end_address: u16,
};

// adjust to your real signatures
fn dummyRead(mem: []u8, offset: u8) u8 {
    return mem[offset];
}
fn dummyWrite(mem: []u8, offset: u8, value: u8) void {
    mem[offset] = value;
}

// this is now comptime-known
pub const MEMORY_MAP = [_]MMIO{
    // 0x0000–0x3FFF: fixed ROM
    .{
        .name = "ROM0",
        .read = cart_mmio.rom0Read,
        .write = cart_mmio.rom0Write,
        .start_address = 0x0000,
        .end_address = 0x3FFF,
    },
    // 0x4000–0x7FFF: switchable ROM
    .{
        .name = "ROMX",
        .read = cart_mmio.romxRead,
        .write = cart_mmio.romxWrite,
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
        .read = ioRead,
        .write = ioWrite,
        .start_address = 0xFF00,
        .end_address = 0xFFFF,
    },
};
