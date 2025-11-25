const std = @import("std");

/// # OAM Summary
/// object attribute memory: this is where object attributes reside
/// each of the 40 possible sprites that can exist consist of four bytes...
pub const SPRITE_COUNT: usize = 40;
pub const ENTRY_SIZE: usize = 4;
pub const OAM_SIZE: usize = SPRITE_COUNT * ENTRY_SIZE;

pub const Sprite = packed struct {
    y: u8, // y position + 16
    x: u8, // x position + 8
    tile: u8, // tile index
    flags: u8, // flags per bit (attributes)
};

comptime {
    std.debug.assert(@sizeOf(Sprite) == ENTRY_SIZE);
    std.debug.assert(OAM_SIZE == SPRITE_COUNT * @sizeOf(Sprite));
}

/// $FE00–$FE9F is the memory mapped region for OAM!
pub const Oam = struct {
    // 160 bytes of the memory that represents the OAM
    mem: [OAM_SIZE]u8 = [_]u8{0} ** OAM_SIZE,
    ppu_mode: *const u8 = undefined,

    /// initializes OAM based on PPU mode
    pub fn init(self: *Oam, ppu_mode: *const u8) void {
        self.ppu_mode = ppu_mode;
        self.reset();
    }

    /// converts the entire OAM into a Sprite slice
    pub fn sprites(self: *Oam) []align(1) Sprite {
        return std.mem.bytesAsSlice(Sprite, self.mem[0..]);
    }

    /// checks if ppu mode allows for CPU reads and writes!
    fn accessAllowed(self: *Oam) bool {
        const mode = self.ppu_mode.*;
        // CPU is blocked in modes 2 (OAM scan) and 3 (transfer)
        return !(mode == 2 or mode == 3);
    }

    /// set all OAM bytes to 0
    pub fn reset(self: *Oam) void {
        @memset(self.mem[0..], 0);
    }

    /// CPU-facing: return byte of memory at offset
    pub fn read(self: *Oam, mem: []u8, off: u8) u8 {
        _ = mem; // slice from bus; we store separately in self.mem

        // assert offset is within OAM range
        std.debug.assert(off < self.mem.len);

        if (!self.accessAllowed())
            return 0xFF;

        return self.mem[off];
    }

    /// CPU-facing: write byte of memory at offset
    pub fn write(self: *Oam, mem: []u8, off: u8, val: u8) void {
        _ = mem; // slice from bus; we store separately in self.mem

        // assert offset is within OAM range
        std.debug.assert(off < self.mem.len);

        if (!self.accessAllowed())
            return;

        self.mem[off] = val;
    }

    /// get sprite entry at the index
    pub fn getSprite(self: *Oam, idx: usize) Sprite {
        return self.sprites()[idx];
    }

    /// called by PPU to select up to 10 sprites for a scanline
    pub fn selectScanlineSprites(
        self: *Oam,
        allocator: std.mem.Allocator,
        ly: u8, // current scanline (LY)
        sprite_height: u8, // 8 or 16 according to LCDC
    ) ![]Sprite {
        // allocate space for up to 10 sprites
        var buf = try allocator.alloc(Sprite, 10);
        var count: usize = 0;

        // selection priority = OAM order
        for (self.sprites()) |s| {
            const y_top = @as(i16, s.y) - 16;
            const y_bottom = y_top + @as(i16, sprite_height);
            if (@as(i16, ly) >= y_top and @as(i16, ly) < y_bottom) {
                buf[count] = s;
                count += 1;
                if (count >= 10) break;
            }
        }

        // return only the used prefix
        return buf[0..count];
    }
};

/// helper that sets the parts of mem to given values for the test cases
fn setEntry(o: *Oam, idx: usize, y: u8, x: u8, tile: u8, flags: u8) void {
    const base = idx * ENTRY_SIZE;
    o.mem[base + 0] = y;
    o.mem[base + 1] = x;
    o.mem[base + 2] = tile;
    o.mem[base + 3] = flags;
}

test "OAM: reset zeroes all 160 bytes" {
    var mode: u8 = 0;
    var o = Oam{};
    o.init(&mode);

    // dirty it first
    for (o.mem[0..], 0..) |*b, i| {
        b.* = @as(u8, @intCast((i * 3) & 0xFF));
    }

    o.reset();

    for (o.mem) |b| {
        try std.testing.expectEqual(@as(u8, 0), b);
    }
}

test "OAM: read/write on single bytes with mode gating" {
    var mode: u8 = 0;
    var o = Oam{};
    o.init(&mode);

    // allowed in mode 0
    o.write(&[_]u8{}, 0x00, 0x12);
    o.write(&[_]u8{}, 0x9F, 0xAB);
    try std.testing.expectEqual(@as(u8, 0x12), o.read(&[_]u8{}, 0x00));
    try std.testing.expectEqual(@as(u8, 0xAB), o.read(&[_]u8{}, 0x9F));

    // switch to forbidden mode (2)
    mode = 2;
    o.write(&[_]u8{}, 0x00, 0x34); // should be ignored
    try std.testing.expectEqual(@as(u8, 0x12), o.mem[0x00]);
    try std.testing.expectEqual(@as(u8, 0xFF), o.read(&[_]u8{}, 0x00));
}

test "OAM: selectScanlineSprites caps at 10 and preserves OAM order" {
    var mode: u8 = 0;
    var o = Oam{};
    o.init(&mode);

    const ly: u8 = 50;

    // put 12 sprites overlapping LY=50
    for (0..@min(12, SPRITE_COUNT)) |i| {
        setEntry(
            &o,
            i,
            ly + 16, // top at this scanline (y field is +16)
            @as(u8, @intCast(i + 8)), // x
            @as(u8, @intCast(i)), // tile = i (to test order)
            0,
        );
    }

    const list = try o.selectScanlineSprites(std.testing.allocator, ly, 8);
    defer std.testing.allocator.free(list);

    // hardware limit: 10
    try std.testing.expectEqual(@as(usize, 10), list.len);

    // order should match OAM order (tile = i)
    for (list, 0..) |sp, i| {
        try std.testing.expectEqual(@as(u8, @intCast(i)), sp.tile);
    }
}
