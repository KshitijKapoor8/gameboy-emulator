const std = @import("std");
const Bus64KB = @import("bus.zig").Bus64KB;

const expect = std.testing.expect;

pub const CPU = struct {
    // accumulator and flags
    A: u8,
    F: u8, // flags: Z N H C -> bits 7, 6, 5, 4 respectively

    // -- general purpose registers --
    // B is high, C is low
    B: u8,
    C: u8,
    // D is high, E is low
    D: u8,
    E: u8,
    // H is high, L is low
    H: u8,
    L: u8,

    // stack pointer, program counter, and is CPU halted
    SP: u16,
    PC: u16,
    halted: bool = false,

    // flag masks
    pub const Z_FLAG: u8 = 0b1000_0000;
    pub const N_FLAG: u8 = 0b0100_0000;
    pub const H_FLAG: u8 = 0b0010_0000;
    pub const C_FLAG: u8 = 0b0001_0000;

    // set the F register with given mask, zero out bottom bits
    inline fn setFlag(self: *CPU, mask: u8, on: bool) void {
        if (on) self.F |= mask else self.F &= ~mask;
        self.F &= 0xF0;
    }

    // get flag from F register
    inline fn getFlag(self: *const CPU, mask: u8) bool {
        return (self.F & mask) != 0;
    }

    // helper to read HL as a 16-bit address
    // H is the upper 8 bits of an address, L is the lower 8 bits of an address
    inline fn getHL(self: *const CPU) u16 {
        return (@as(u16, self.H) << 8) | self.L;
    }

    pub fn step(self: *CPU, bus: *Bus64KB()) u32 {
        if (self.halted) return 4;

        const op = bus.read(self.PC);
        self.PC +%= 1;

        switch (op) {
            0x00 => return 4,

            // Ox0E LD C, d8
            0x0E => {
                self.C = bus.read(self.PC);
                self.PC +%= 1;
                return 8;
            },
            // 0x1E LD E, d8
            0x1E => {
                self.E = bus.read(self.PC);
                self.PC +%= 1;
                return 8;
            },
            // 0x2E LD L, d8
            0x2E => {
                self.L = bus.read(self.PC);
                self.PC +%= 1;
                return 8;
            },

            // 0x3E LD A, d8
            0x3E => {
                self.A = bus.read(self.PC);
                self.PC +%= 1;
                return 8;
            },

            // 0x4E LD C,(HL)
            0x4E => {
                const val = bus.read(self.getHL());
                self.C = val;
                return 8;
            },

            // 0x5E LD E,(HL)
            0x5E => {
                const val = bus.read(self.getHL());
                self.E = val;
                return 8;
            },

            // 0x6E LD L,(HL)
            0x6E => {
                const val = bus.read(self.getHL());
                self.L = val;
                return 8;
            },

            // 0x7E LD A,(HL)
            0x7E => {
                const val = bus.read(self.getHL());
                self.A = val;
                return 8;
            },
        }
    }
};
