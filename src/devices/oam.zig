const std = @import("std");

/// # OAM Summary
/// object attribute memory:  this is where object attributes reside
/// each of the 40 possible sprites that can exist consist of four bytes...
pub const SPRITE_COUNT = 40;
pub const ENTRY_SIZE = 4;
pub const OAM_SIZE = SPRITE_COUNT * ENTRY_SIZE;

pub const Sprite = packed struct {
    y: u8, // yposition + 16
    x: u8, // x position + 8
    tile: u8, // tile index
    flags: u8, // this byte has a bunch of flags per bit, should be outlined in the gbdev.io 
};

// $FE00–$FE9F is the memory mapped region for OAM!
pub const Oam = struct {
    // 160 bytes of the memory that represents the OAM
    mem: [OAM_SIZE]u8 = [_]u8{0} ** OAM_SIZE,

    // converts the entire OAM into a Sprite
    pub fn sprites(self: *Oam) []Sprite {
        return std.mem.bytesAsSlice(Sprite, self.mem[0..]);
    }

    // set all oam bytes to 0
    pub fn reset(self: *Oam) void {
        std.mem.set(u8, self.mem[0..], 0);
    }

    // return byte of memory at offset
    pub fn read(self: *Oam, mem: []u8, off: u8) u8 {
        _ = mem; // nonsense but we pass it in in the general devices.zig lol
        return self.mem[off];
    }

    // write byte of memory at offset
    pub fn write(self: *Oam, mem: []u8, off: u8, val: u8) void {
        _ = mem; // nonsense but we pass it in in the general devices.zig lol
        self.mem[off] = val;
    }

    // get spite entry at the index
    pub fn getSprite(self: *Oam, idx: usize) Sprite {
        return self.sprites()[idx];
    }

    // will be called by PPU to select up to 10 sprites, selection priority determined by PPU
    pub fn selectScanlineSprites(
        self: *Oam,
        allocator: std.mem.Allocator,
        ly: u8, // given by PPU during scanline's OAM scan
        sprite_height: u8, // will be 8 or 16 according to the docs
    ) ![]Sprite {
        var selected = std.ArrayList(Sprite).init(allocator);
        var count: usize = 0;

        for (self.sprites()) |s| {
            const y_top = @as(i16, s.y) - 16;
            const y_bottom = y_top + @as(i16, sprite_height);
            if (@as(i16, ly) >= y_top and @as(i16, ly) < y_bottom) {
                try selected.append(s);
                count += 1;
                if (count >= 10) 
                    break;
            }
        }
        return selected.toOwnedSlice();
    }
};

// test cases time

// helper that sets the parts of mem to given values for the test cases
fn setEntry(o: *Oam, idx: usize, y: u8, x: u8, tile: u8, flags: u8) void {
    const base = idx * ENTRY_SIZE;
    o.mem[base + 0] = y;
    o.mem[base + 1] = x;
    o.mem[base + 2] = tile;
    o.mem[base + 3] = flags;
}

test "OAM: reset zeroes all 160 bytes" {
    // make instance of stuct
    var o = Oam{};
    
    for (o.mem[0..], 0..) |*b, i| 
    {
        b.* = @intCast(((i * 3) & 0xFF).*);
    }
        
    o.reset();
    for (o.mem) |b| {
        try std.testing.expectEqual(@as(u8, 0), b);
    }
}

test "OAM: read/write on single bytes" {
    var o = Oam{};
    // mem is pretty useless in OAM context so just pass in empty
    o.write(&[_]u8{}, 0x00, 0x12);
    o.write(&[_]u8{}, 0x9F, 0xAB);
    try std.testing.expectEqual(@as(u8, 0x12), o.read(&[_]u8{}, 0x00));
    try std.testing.expectEqual(@as(u8, 0xAB), o.read(&[_]u8{}, 0x9F));
}

test "OAM: selectScanlineSprites caps at 10 and preserves OAM order" {
    var o = Oam{};
    // put 12 sprites on LY=50
    const ly: u8 = 50;
    for (std.math.min(12, SPRITE_COUNT)) |i| {
        setEntry(&o, i, ly + 16, @intCast(i + 8), @intCast(i), 0);
    }

    const list = try o.selectScanlineSprites(std.testing.allocator, ly, 8);
    defer std.testing.allocator.free(list);

    try std.testing.expectEqual(@as(usize, 10), list.len); // hardware limit
    // order should be same as in oam
    for (list, 0..) |sp, i| try std.testing.expectEqual(@as(u8, @intCast(i)), sp.tile);
}
