const std = @import("std");

pub const Vram = struct {
    mem: []u8,
    ppu_mode: *const u8,

    pub fn read(self: *Vram, off: u8) u8 {
        const mode = self.ppu_mode.*;
        if (mode == 3) return 0xFF;
        return self.mem[off];
    }

    pub fn write(self: *Vram, off: u8, v: u8) void {
        const mode = self.ppu_mode.*;
        if (mode == 3) return;
        self.mem[off] = v;
    }
};

test "VRAM: basic read/write works in mode 0" {
    var mode: u8 = 0;

    var backing: [0x2000]u8 = [_]u8{0} ** 0x2000;
    var vram = Vram{
        .mem = backing[0..],
        .ppu_mode = &mode,
    };

    vram.write(0x10, 0xAB);
    try std.testing.expectEqual(@as(u8, 0xAB), vram.read(0x10));
}

test "VRAM: mode 3 blocks writes" {
    var mode: u8 = 3;

    var backing: [0x2000]u8 = [_]u8{0} ** 0x2000;
    var vram = Vram{
        .mem = backing[0..],
        .ppu_mode = &mode,
    };

    vram.mem[0x20] = 0x11; // preset value
    vram.write(0x20, 0xFF); // should be ignored

    try std.testing.expectEqual(@as(u8, 0x11), vram.mem[0x20]);
}

test "VRAM: mode 3 blocks reads" {
    var mode: u8 = 3;

    var backing: [0x2000]u8 = [_]u8{0} ** 0x2000;
    var vram = Vram{
        .mem = backing[0..],
        .ppu_mode = &mode,
    };

    vram.mem[0x30] = 0x77; // actual value shouldn't matter
    const val = vram.read(0x30);

    try std.testing.expectEqual(@as(u8, 0xFF), val);
}
