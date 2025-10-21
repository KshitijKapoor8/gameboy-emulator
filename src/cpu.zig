const std = @import("std");

// GB has a 64Kb Bus
pub const Bus = struct {
    mem: [0x10000]u8 = .{0} ** 0x10000,
    pub fn read(self: *const Bus, addr: u16) u8 {
        return @intCast(self.mem[addr]);
    }
    pub fn write(self: *Bus, addr: u16, val: u8) void {
        self.mem[addr] = val;
    }
};

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

    // TODO: verify
    pub fn step(self: *CPU, bus: *Bus) u32 {
        if (self.halted) return 4;

        const op = bus.read(self.PC);
        self.PC +%= 1;

        switch (op) {
            0x00 => return 4,

            // 0x3E LD A, d8
            0x3E => {
                self.A = bus.read(self.PC);
                self.PC +%= 1;
                return 8;
            },
        }
    }
};
