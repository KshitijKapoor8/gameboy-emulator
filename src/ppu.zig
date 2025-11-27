const std = @import("std");

/// Game Boy PPU (Picture Processing Unit)
/// Handles rendering graphics to a 160x144 display
pub const PPU = struct {
    // Display dimensions
    pub const SCREEN_WIDTH: u32 = 160;
    pub const SCREEN_HEIGHT: u32 = 144;
    pub const FRAMEBUFFER_SIZE: usize = SCREEN_WIDTH * SCREEN_HEIGHT * 4; // RGBA

    // PPU Modes
    pub const Mode = enum(u8) {
        HBlank = 0, // Horizontal blank - CPU can access VRAM/OAM
        VBlank = 1, // Vertical blank - CPU can access VRAM/OAM
        OAMScan = 2, // Scanning OAM - OAM locked
        Drawing = 3, // Drawing pixels - VRAM/OAM locked
    };

    // Mode timing in dots (1 dot = 1 M-cycle = 4 T-cycles)
    pub const DOTS_PER_LINE = 456;
    pub const OAM_SCAN_DOTS = 80;
    pub const DRAWING_DOTS = 172; // minimum, can be longer
    pub const HBLANK_DOTS = 204; // remaining dots
    pub const LINES_PER_FRAME = 154; // 144 visible + 10 vblank
    pub const VBLANK_LINES = 10;

    // LCD Control Register (0xFF40) flags
    pub const LCDC_BG_ENABLE: u8 = 0b0000_0001;
    pub const LCDC_OBJ_ENABLE: u8 = 0b0000_0010;
    pub const LCDC_OBJ_SIZE: u8 = 0b0000_0100; // 0=8x8, 1=8x16
    pub const LCDC_BG_TILEMAP: u8 = 0b0000_1000; // 0=9800-9BFF, 1=9C00-9FFF
    pub const LCDC_BG_TILE_DATA: u8 = 0b0001_0000; // 0=8800-97FF, 1=8000-8FFF
    pub const LCDC_WIN_ENABLE: u8 = 0b0010_0000;
    pub const LCDC_WIN_TILEMAP: u8 = 0b0100_0000; // 0=9800-9BFF, 1=9C00-9FFF
    pub const LCDC_LCD_ENABLE: u8 = 0b1000_0000;

    // LCD Status Register (0xFF41) flags
    pub const STAT_MODE_FLAG: u8 = 0b0000_0011;
    pub const STAT_LYC_EQ_LY: u8 = 0b0000_0100;
    pub const STAT_HBLANK_INT: u8 = 0b0000_1000;
    pub const STAT_VBLANK_INT: u8 = 0b0001_0000;
    pub const STAT_OAM_INT: u8 = 0b0010_0000;
    pub const STAT_LYC_INT: u8 = 0b0100_0000;

    // Grayscale palette (DMG Game Boy)
    pub const COLORS = [4][3]u8{
        .{ 0xFF, 0xFF, 0xFF }, // White
        .{ 0xAA, 0xAA, 0xAA }, // Light gray
        .{ 0x55, 0x55, 0x55 }, // Dark gray
        .{ 0x00, 0x00, 0x00 }, // Black
    };

    // Current state
    mode: Mode,
    dots: u32, // Dots elapsed in current scanline
    ly: u8, // Current scanline (0-153)
    frame_complete: bool,

    // Framebuffer (RGBA format)
    framebuffer: [FRAMEBUFFER_SIZE]u8,

    // Interrupt flags
    vblank_interrupt: bool,
    stat_interrupt: bool,

    pub fn init() PPU {
        return .{
            .mode = .OAMScan,
            .dots = 0,
            .ly = 0,
            .frame_complete = false,
            .framebuffer = [_]u8{0xFF} ** FRAMEBUFFER_SIZE, // Start with white
            .vblank_interrupt = false,
            .stat_interrupt = false,
        };
    }

    /// Step the PPU by a number of dots (M-cycles)
    /// Returns true if a frame was completed
    pub fn step(self: *PPU, dots: u32, vram: []const u8, lcdc: u8, stat: u8, scy: u8, scx: u8, bgp: u8) bool {
        if ((lcdc & LCDC_LCD_ENABLE) == 0) {
            // LCD is off, reset state
            self.ly = 0;
            self.dots = 0;
            self.mode = .HBlank;
            return false;
        }

        self.dots += dots;
        self.frame_complete = false;

        // Process based on current mode
        switch (self.mode) {
            .OAMScan => {
                if (self.dots >= OAM_SCAN_DOTS) {
                    self.mode = .Drawing;
                    self.dots -= OAM_SCAN_DOTS;
                }
            },
            .Drawing => {
                if (self.dots >= DRAWING_DOTS) {
                    // Render the current scanline
                    if (self.ly < SCREEN_HEIGHT) {
                        self.renderScanline(vram, lcdc, scy, scx, bgp);
                    }

                    self.mode = .HBlank;
                    self.dots -= DRAWING_DOTS;

                    // Check for HBLANK interrupt
                    if ((stat & STAT_HBLANK_INT) != 0) {
                        self.stat_interrupt = true;
                    }
                }
            },
            .HBlank => {
                if (self.dots >= HBLANK_DOTS) {
                    self.dots -= HBLANK_DOTS;
                    self.ly += 1;

                    if (self.ly >= SCREEN_HEIGHT) {
                        // Enter VBlank
                        self.mode = .VBlank;
                        self.vblank_interrupt = true;
                        self.frame_complete = true;

                        // Check for VBLANK interrupt
                        if ((stat & STAT_VBLANK_INT) != 0) {
                            self.stat_interrupt = true;
                        }
                    } else {
                        // Next scanline
                        self.mode = .OAMScan;

                        // Check for OAM interrupt
                        if ((stat & STAT_OAM_INT) != 0) {
                            self.stat_interrupt = true;
                        }
                    }
                }
            },
            .VBlank => {
                if (self.dots >= DOTS_PER_LINE) {
                    self.dots -= DOTS_PER_LINE;
                    self.ly += 1;

                    if (self.ly >= LINES_PER_FRAME) {
                        // Frame complete, restart
                        self.ly = 0;
                        self.mode = .OAMScan;

                        // Check for OAM interrupt
                        if ((stat & STAT_OAM_INT) != 0) {
                            self.stat_interrupt = true;
                        }
                    }
                }
            },
        }

        return self.frame_complete;
    }

    /// Render a single scanline to the framebuffer
    fn renderScanline(self: *PPU, vram: []const u8, lcdc: u8, scy: u8, scx: u8, bgp: u8) void {
        if ((lcdc & LCDC_BG_ENABLE) == 0) {
            // Background disabled, fill with white
            const y = self.ly;
            for (0..SCREEN_WIDTH) |x| {
                self.setPixel(@intCast(x), y, 0); // Color 0 = white
            }
            return;
        }

        // Determine which tile map to use
        const tilemap_base: u16 = if ((lcdc & LCDC_BG_TILEMAP) != 0) 0x1C00 else 0x1800; // Relative to VRAM start

        // Determine which tile data to use
        const tile_data_base: u16 = if ((lcdc & LCDC_BG_TILE_DATA) != 0) 0x0000 else 0x0800;
        const signed_addressing = (lcdc & LCDC_BG_TILE_DATA) == 0;

        const y = self.ly;
        const scroll_y = @as(u16, scy) +% y;
        const tile_row = scroll_y / 8;

        for (0..SCREEN_WIDTH) |screen_x| {
            const x: u8 = @intCast(screen_x);
            const scroll_x = @as(u16, scx) +% x;
            const tile_col = scroll_x / 8;

            // Get tile index from tile map
            const tilemap_addr = tilemap_base + (tile_row % 32) * 32 + (tile_col % 32);
            const tile_index = vram[tilemap_addr];

            // Calculate tile data address
            var tile_addr: u16 = undefined;
            if (signed_addressing) {
                const signed_index: i8 = @bitCast(tile_index);
                tile_addr = tile_data_base + @as(u16, @intCast(@as(i32, signed_index) + 128)) * 16;
            } else {
                tile_addr = tile_data_base + @as(u16, tile_index) * 16;
            }

            // Get pixel within tile
            const tile_y = scroll_y % 8;
            const tile_x = scroll_x % 8;

            // Each tile line is 2 bytes
            const line_addr = tile_addr + tile_y * 2;
            const byte1 = vram[line_addr];
            const byte2 = vram[line_addr + 1];

            // Extract pixel color (bit 0 of each byte, MSB first)
            const bit_pos: u3 = @intCast(7 - (tile_x % 8));
            const color_low: u8 = (byte1 >> bit_pos) & 1;
            const color_high: u8 = (byte2 >> bit_pos) & 1;
            const color_id = (color_high << 1) | color_low;

            // Apply palette
            const palette_shift: u3 = @intCast(color_id * 2);
            const palette_color = (bgp >> palette_shift) & 0x03;

            self.setPixel(x, y, palette_color);
        }
    }

    /// Set a pixel in the framebuffer
    fn setPixel(self: *PPU, x: u8, y: u8, color_id: u8) void {
        const offset = (@as(usize, y) * SCREEN_WIDTH + @as(usize, x)) * 4;
        const color = COLORS[color_id & 0x03];
        self.framebuffer[offset + 0] = color[0]; // R
        self.framebuffer[offset + 1] = color[1]; // G
        self.framebuffer[offset + 2] = color[2]; // B
        self.framebuffer[offset + 3] = 0xFF; // A
    }

    /// Get the current mode for STAT register
    pub fn getMode(self: *const PPU) u8 {
        return @intFromEnum(self.mode);
    }

    /// Get current scanline (LY register)
    pub fn getLY(self: *const PPU) u8 {
        return self.ly;
    }

    /// Check and clear VBlank interrupt flag
    pub fn checkVBlankInterrupt(self: *PPU) bool {
        const result = self.vblank_interrupt;
        self.vblank_interrupt = false;
        return result;
    }

    /// Check and clear STAT interrupt flag
    pub fn checkStatInterrupt(self: *PPU) bool {
        const result = self.stat_interrupt;
        self.stat_interrupt = false;
        return result;
    }

    /// Get pointer to framebuffer for rendering
    pub fn getFramebuffer(self: *const PPU) []const u8 {
        return &self.framebuffer;
    }
};

test "PPU initialization" {
    const ppu = PPU.init();
    try std.testing.expectEqual(PPU.Mode.OAMScan, ppu.mode);
    try std.testing.expectEqual(@as(u8, 0), ppu.ly);
    try std.testing.expectEqual(@as(u32, 0), ppu.dots);
}

test "PPU mode transitions" {
    var ppu = PPU.init();
    var vram = [_]u8{0} ** 0x2000;

    // Start in OAMScan
    try std.testing.expectEqual(PPU.Mode.OAMScan, ppu.mode);

    // Step through OAM scan (80 dots)
    _ = ppu.step(80, &vram, PPU.LCDC_LCD_ENABLE | PPU.LCDC_BG_ENABLE, 0, 0, 0, 0xE4);
    try std.testing.expectEqual(PPU.Mode.Drawing, ppu.mode);

    // Step through drawing (172 dots minimum)
    _ = ppu.step(172, &vram, PPU.LCDC_LCD_ENABLE | PPU.LCDC_BG_ENABLE, 0, 0, 0, 0xE4);
    try std.testing.expectEqual(PPU.Mode.HBlank, ppu.mode);

    // Step through HBlank (204 dots)
    _ = ppu.step(204, &vram, PPU.LCDC_LCD_ENABLE | PPU.LCDC_BG_ENABLE, 0, 0, 0, 0xE4);
    try std.testing.expectEqual(PPU.Mode.OAMScan, ppu.mode);
    try std.testing.expectEqual(@as(u8, 1), ppu.ly);
}

test "PPU enters VBlank after line 143" {
    var ppu = PPU.init();
    var vram = [_]u8{0} ** 0x2000;

    // Fast-forward to line 143
    ppu.ly = 143;
    ppu.mode = .HBlank;
    ppu.dots = 0;

    // Complete line 143
    _ = ppu.step(PPU.HBLANK_DOTS, &vram, PPU.LCDC_LCD_ENABLE | PPU.LCDC_BG_ENABLE, 0, 0, 0, 0xE4);

    try std.testing.expectEqual(PPU.Mode.VBlank, ppu.mode);
    try std.testing.expectEqual(@as(u8, 144), ppu.ly);
    try std.testing.expect(ppu.vblank_interrupt);
}

test "PPU disabled state" {
    var ppu = PPU.init();
    var vram = [_]u8{0} ** 0x2000;

    ppu.ly = 100;
    _ = ppu.step(100, &vram, 0, 0, 0, 0, 0xE4); // LCDC = 0 (LCD disabled)

    try std.testing.expectEqual(@as(u8, 0), ppu.ly);
    try std.testing.expectEqual(PPU.Mode.HBlank, ppu.mode);
}
