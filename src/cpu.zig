const std = @import("std");
const SystemBus = @import("system_bus.zig").SystemBus;
// const g_test_system_bus = @import("system_bus.zig").g_test_system_bus;

const expect = std.testing.expect;

pub const CPU = struct {
    A: u8,
    F: u8, // flags: Z N H C -> bits 7, 6, 5, 4 respectively

    // -- general purpose registers, high then low --
    B: u8,
    C: u8,

    D: u8,
    E: u8,

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

    /// # Set a specific flag using a flag mask
    ///
    /// Either sets or removes a CPU flag, useful for instruction processing
    ///
    /// # Parameters
    /// - `self`: a reference to the CPU
    /// - `mask`: defines the specific flag to set
    /// - `on`: whether to set the flag to 1 or 0
    inline fn setFlag(self: *CPU, mask: u8, on: bool) void {
        if (on) self.F |= mask else self.F &= ~mask;
        self.F &= 0xF0;
    }

    /// # Get a specific flag using a flag mask from the flag register
    ///
    /// # Parameters
    /// - `self`: a reference to the CPU
    /// - `mask`: defines the specific flag to get
    ///
    /// # Returns
    /// The value of the flag as true or false
    inline fn getFlag(self: *const CPU, mask: u8) bool {
        return (self.F & mask) != 0;
    }

    /// # Get a 16 bit register
    ///
    /// # Parameters
    /// - `high`: an 8-bit register that corresponds to the high bits of the 16 bit register
    /// - `low`: an 8-bit register that corresponds to the high bits of the 16 bit register
    ///
    /// # Returns
    /// A u16 that is the value of the 16-bit register
    inline fn get16BitRegister(high: u8, low: u8) u16 {
        return (@as(u16, high) << 8) | low;
    }

    /// # Set a 16-bit register pair
    ///
    /// # Parameters
    /// - `high`: pointer to the high 8-bit register
    /// - `low`: pointer to the low 8-bit register
    /// - `value`: the 16-bit value to write
    ///
    /// # Behavior
    /// Splits `value` into high and low bytes and stores them.
    inline fn set16BitRegister(
        high: *u8,
        low: *u8,
        value: u16,
    ) void {
        high.* = @intCast(value >> 8);
        low.* = @intCast(value & 0xFF);
    }

    /// # Fetch, decode, and execute one instruction
    ///
    /// # Parameters
    /// - `self`: a reference to the CPU struct that contains the CPU state to modify
    /// - `system_bus`: a reference to the system bus in case you need to interface with memory
    ///
    /// # Returns
    /// A u32 that reflects the number of CPU cycles needed to run the instruction, or an error if something went wrong
    pub fn step(self: *CPU, system_bus: *SystemBus) !u32 {
        if (self.halted) return 4;

        const op = try system_bus.read(self.PC);
        self.PC +%= 1;

        switch (op) {
            // no-op
            0x00 => return 4,

            // LD BC, n16
            0x01 => {
                const low = try system_bus.read(self.PC);
                self.PC +%= 1;
                const high = try system_bus.read(self.PC);
                self.PC +%= 1;

                self.B = high;
                self.C = low;

                return 12;
            },

            // LD [BC], A
            0x02 => {
                system_bus.write(get16BitRegister(self.H, self.L), self.A);
                return 8;
            },

            // INC BC
            0x03 => {
                var BC = get16BitRegister(self.B, self.C);
                BC +%= 1;
                set16BitRegister(&self.B, &self.C, BC);
                return 8;
            },

            // INC B
            0x04 => {
                const old = self.B;
                self.B +%= 1;
                self.setFlag(Z_FLAG, self.B == 0);
                self.setFlag(N_FLAG, 0);
                self.setFlag(H_FLAG, (old & 0x0F) + 1 > 0x0F);

                return 4;
            },

            // DEC B
            0x05 => {
                const old = self.B;
                self.B -%= 1;
                self.setFlag(Z_FLAG, self.B == 0);
                self.setFlag(N_FLAG, 1);
                self.setFlag(H_FLAG, (old & 0x0F) == 0x00);

                return 4;
            },

            // LD B, n8
            0x06 => {
                self.B = try system_bus.read(self.PC);
                self.PC +%= 1;
                return 8;
            },

            // Ox0E LD C, d8
            0x0E => {
                self.C = try system_bus.read(self.PC);
                self.PC +%= 1;
                return 8;
            },
            // 0x1E LD E, d8
            0x1E => {
                self.E = try system_bus.read(self.PC);
                self.PC +%= 1;
                return 8;
            },
            // 0x2E LD L, d8
            0x2E => {
                self.L = try system_bus.read(self.PC);
                self.PC +%= 1;
                return 8;
            },

            // 0x3E LD A, d8
            0x3E => {
                self.A = try system_bus.read(self.PC);
                self.PC +%= 1;
                return 8;
            },

            // 0x4E LD C,(HL)
            0x4E => {
                const val = try system_bus.read(CPU.get16BitRegister(self.H, self.L));
                self.C = val;
                return 8;
            },

            // 0x5E LD E,(HL)
            0x5E => {
                const val = try system_bus.read(CPU.get16BitRegister(self.H, self.L));
                self.E = val;
                return 8;
            },

            // 0x6E LD L,(HL)
            0x6E => {
                const val = try system_bus.read(CPU.get16BitRegister(self.H, self.L));
                self.L = val;
                return 8;
            },

            // 0x7E LD A,(HL)
            0x7E => {
                const val = try system_bus.read(CPU.get16BitRegister(self.H, self.L));
                self.A = val;
                return 8;
            },

            else => {
                return error.UnimplementedOpcode;
            },
        }
    }
};

test "CPU runs a tiny GB-like program" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);
    // program at 0x0100, like real GB after boot
    // 0x0E d8  -> LD C, imm8
    try bus.write(0x0100, 0x0E);
    try bus.write(0x0101, 0x12);

    // 0x3E d8  -> LD A, imm8
    try bus.write(0x0102, 0x3E);
    try bus.write(0x0103, 0x34);

    // 0x7E     -> LD A, (HL)
    try bus.write(0x0104, 0x7E);

    // pretend WRAM at 0xC000 has some data
    try bus.write(0xC000, 0xAB);

    // set up CPU like real GB after boot ROM:
    // PC = 0x0100, SP often 0xFFFE, HL we'll set to 0xC000
    var cpu = CPU{
        .A = 0,
        .F = 0,
        .B = 0,
        .C = 0,
        .D = 0,
        .E = 0,
        .H = 0xC0,
        .L = 0x00,
        .SP = 0xFFFE,
        .PC = 0x0100,
        .halted = false,
    };

    // 1) LD C, 0x12
    var cycles = try cpu.step(&bus);
    try expect(cycles == 8); // LD r, d8 is 8 cycles
    try expect(cpu.C == 0x12);
    try expect(cpu.PC == 0x0102);

    // 2) LD A, 0x34
    cycles = try cpu.step(&bus);
    try expect(cycles == 8);
    try expect(cpu.A == 0x34);
    try expect(cpu.PC == 0x0104);

    // 3) LD A, (HL) -> HL = 0xC000 -> mem = 0xAB
    cycles = try cpu.step(&bus);
    try expect(cycles == 8);
    try expect(cpu.A == 0xAB);
    try expect(cpu.PC == 0x0105);
}
