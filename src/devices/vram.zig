const std = @import("std");

var ppu_mode_ptr: *const u8 = undefined; // ppu mode some int 0-3 there are 4 ppu modes

pub const Config = struct {
    ppu_mode: *const u8,
};

pub fn init(cfg: Config) void {
    ppu_mode_ptr = cfg.ppu_mode;
}

pub fn read(mem: []u8, off: u8) u8 {
    const mode = ppu_mode_ptr.*;
    if (mode == 3)
        return 0xFF; // print nonsense, cannot read in mode 3
    return mem[off];
}

pub fn write(mem: []u8, off: u8, v: u8) void {
    const mode = ppu_mode_ptr.*;
    if (mode == 3)
        return; // refuse to allow write in mode 3
    mem[off] = v;
}

test "VRAM: basic read/write works in mode 0" {
    var mode: u8 = 0;
    init(.{ .ppu_mode = &mode });

    var backing: [0x2000]u8 = [_]u8{0} ** 0x2000;
    const mem = backing[0..];

    write(mem, 0x10, 0xAB);
    try std.testing.expectEqual(@as(u8, 0xAB), read(mem, 0x10));
}

test "VRAM: mode 3 blocks writes" {
    var mode: u8 = 3;
    init(.{ .ppu_mode = &mode });

    var backing: [0x2000]u8 = [_]u8{0} ** 0x2000;
    const mem = backing[0..];

    mem[0x20] = 0x11; // preset value
    write(mem, 0x20, 0xFF); // should be ignored

    try std.testing.expectEqual(@as(u8, 0x11), mem[0x20]);
}

test "VRAM: mode 3 blocks reads" {
    var mode: u8 = 3;
    init(.{ .ppu_mode = &mode });

    var backing: [0x2000]u8 = [_]u8{0} ** 0x2000;
    const mem = backing[0..];

    mem[0x30] = 0x77; // actual value shouldn't matter
    const val = read(mem, 0x30);

    try std.testing.expectEqual(@as(u8, 0xFF), val);
}
