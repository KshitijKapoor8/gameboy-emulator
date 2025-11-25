const std = @import("std");

// joypad has 8 bit input
pub const Joypad = struct {
    // when bit 5 is 0, this will be marked true bc we can assign calues of SsBA
    select_button: bool = false,
    // when bit 4 is 0, this will be marked true bc directional keys can be read in
    select_dpad: bool = false,

    // state is laid out like this: start, select, B, A, down, up, left, right
    state: u8 = 0xFF,

    pub fn read(self: *Joypad) u8 {
        var result: u8 = 0xC0;

        // we asign SsBA if this is true
        if (self.select_button) {
            // laid out like this: Start, Select, B, A and values of these go into buttons
            const buttons = (self.state >> 4) & 0x0F;
            result |= (0x10 | buttons);
        }
        // we assign directions based on state
        else if (self.select_dpad) {
            const dpad = self.state & 0x0F;
            result |= (0x20 | dpad);
        }
        // neither selected !
        else {
            result |= (0x30 | 0x0F);
        }
        return result;
    }

    pub fn write(self: *Joypad, value: u8) void {
        // lower nibble is read only so.. we can only write bits 4 and 5
        self.select_button = (value & 0x20) == 0;
        self.select_dpad = (value & 0x10) == 0;
    }

    // if button is pressed, adjust it in state accordingly
    pub fn press(self: *Joypad, button: Button) void {
        self.state &= ~(@as(u8, 1) << @intFromEnum(button));
    }

    // if button is released, adjust it in state accordingly
    pub fn release(self: *Joypad, button: Button) void {
        self.state |= (@as(u8, 1) << @intFromEnum(button));
    }
};

// button enum
pub const Button = enum(u3) {
    Right = 0,
    Left,
    Up,
    Down,
    A,
    B,
    Select,
    Start,
};

test "power-on: neither group selected reads 0xFF" {
    var jp = Joypad{}; // state=0xFF, no selects
    try std.testing.expectEqual(@as(u8, 0xFF), jp.read());
}

test "select dpad (P14=0) exposes Right/Left/Up/Down in lower nibble" {
    var jp = Joypad{};
    // select dpad only: write with bit5=1, bit4=0 -> 0x20
    jp.write(0x20);
    // all released -> lower nibble 0xF; select bits read back as 0b10 -> 0xE?F -> 0xEF
    try std.testing.expectEqual(@as(u8, 0xEF), jp.read());

    // press Right (bit0 goes 0)
    jp.press(.Right);
    try std.testing.expectEqual(@as(u8, 0xEE), jp.read());

    // press Up (bit2 goes 0)
    jp.press(.Up);
    try std.testing.expectEqual(@as(u8, 0xEA), jp.read());

    // release Right
    jp.release(.Right);
    try std.testing.expectEqual(@as(u8, 0xEB), jp.read());
}

test "select buttons (P15=0) exposes A/B/Select/Start in lower nibble" {
    var jp = Joypad{};
    // select buttons (P15=0)
    jp.write(0x10);
    try std.testing.expectEqual(@as(u8, 0xDF), jp.read());

    jp.press(.A);
    try std.testing.expectEqual(@as(u8, 0xDE), jp.read());

    // press Start (upper nibble: Start=0, Select=1, B=1, A=0 -> 0b0110)
    jp.press(.Start);
    try std.testing.expectEqual(@as(u8, 0xD6), jp.read()); 

    // release A then Start
    jp.release(.A);
    try std.testing.expectEqual(@as(u8, 0xD7), jp.read()); 

    jp.release(.Start);
    try std.testing.expectEqual(@as(u8, 0xDF), jp.read());
}

test "write only affects select bits; not button state" {
    var jp = Joypad{};
    const before = jp.state;
    jp.write(0x10); // change selection
    try std.testing.expectEqual(before, jp.state);
}

test "both selects high (no group) returns 0xFF" {
    var jp = Joypad{};
    jp.write(0x30); // bit5=1, bit4=1 (neither selected)
    try std.testing.expectEqual(@as(u8, 0xFF), jp.read());
}
