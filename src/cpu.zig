const std = @import("std");
const cart_mmio = @import("devices/cart_boot_mmio.zig");
const loader = @import("loader.zig");
const BootRom = @import("devices/bootrom.zig").BootRom;
const Cartridge = @import("devices/cartridge.zig").Cartridge;
const SystemBus = @import("system_bus.zig").SystemBus;
// const g_test_system_bus = @import("system_bus.zig").g_test_system_bus;

const expect = std.testing.expect;

/// Instruction handler function type
const InstructionFn = *const fn (*CPU, *SystemBus) anyerror!u32;

/// Instruction metadata
const Instruction = struct {
    handler: InstructionFn,
    mnemonic: []const u8,
};

/// An implementation of the Sharp LR35902 ISA; defines opcodes and implements instructions
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
    ime: bool = false, // Interrupt Master Enable flag

    // Instruction execution counters
    opcode_counts: [256]u64,
    cb_opcode_counts: [256]u64,

    // flag masks
    pub const Z_FLAG: u8 = 0b1000_0000;
    pub const N_FLAG: u8 = 0b0100_0000;
    pub const H_FLAG: u8 = 0b0010_0000;
    pub const C_FLAG: u8 = 0b0001_0000;

    pub fn init() CPU {
        return .{
            .A = 0,
            .F = 0,
            .B = 0,
            .C = 0,
            .D = 0,
            .E = 0,
            .H = 0,
            .L = 0,
            .SP = 0,
            .PC = 0,
            .halted = false,
            .ime = false,
            .opcode_counts = [_]u64{0} ** 256,
            .cb_opcode_counts = [_]u64{0} ** 256,
        };
    }

    pub fn initPostBoot() CPU {
        return .{
            .A = 0x01,
            .F = 0xB0,
            .B = 0x00,
            .C = 0x13,
            .D = 0x00,
            .E = 0xD8,
            .H = 0x01,
            .L = 0x4D,
            .SP = 0xFFFE,
            .PC = 0x0100,
            .halted = false,
            .ime = false,
            .opcode_counts = [_]u64{0} ** 256,
            .cb_opcode_counts = [_]u64{0} ** 256,
        };
    }

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

    // ==================== Instruction Handlers ====================W

    /// # Generic 8-bit increment with flag handling
    ///
    /// # Parameters
    /// `self` - a reference to the CPU struct
    /// `reg` - the register value to modify
    inline fn inc8(self: *CPU, reg: *u8) void {
        const old = reg.*;
        reg.* +%= 1;
        self.setFlag(Z_FLAG, reg.* == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, (old & 0x0F) + 1 > 0x0F);
    }

    /// # Generic 8-bit decrement with flag handling
    ///
    /// # Parameters
    /// `self` - a reference to the CPU struct
    /// `reg` - the register value to modify
    inline fn dec8(self: *CPU, reg: *u8) void {
        const old = reg.*;
        reg.* -%= 1;
        self.setFlag(Z_FLAG, reg.* == 0);
        self.setFlag(N_FLAG, true);
        self.setFlag(H_FLAG, (old & 0x0F) == 0x00);
    }

    /// Generic 16-bit increment (no flags affected)
    ///
    /// # Parameters
    /// `high` - the higher memory addresses byte
    /// `low` - the lower memory addresses byte
    inline fn inc16(high: *u8, low: *u8) void {
        var val = get16BitRegister(high.*, low.*);
        val +%= 1;
        set16BitRegister(high, low, val);
    }

    /// Generic 16-bit decrement (no flags affected)
    ///
    /// # Parameters
    /// `high` - the higher memory addresses byte
    /// `low` - the lower memory addresses byte
    inline fn dec16(high: *u8, low: *u8) void {
        var val = get16BitRegister(high.*, low.*);
        val -%= 1;
        set16BitRegister(high, low, val);
    }

    /// Read 16-bit immediate value from PC (little-endian: low byte first, then high byte)
    inline fn read16Immediate(self: *CPU, system_bus: *SystemBus) !u16 {
        const low = try system_bus.read(self.PC);
        self.PC +%= 1;
        const high = try system_bus.read(self.PC);
        self.PC +%= 1;
        return (@as(u16, high) << 8) | @as(u16, low);
    }

    /// Generic 8-bit ADD/ADC with carry support and return value
    inline fn add8(self: *CPU, a: u8, b: u8, use_carry: bool) u8 {
        const carry: u16 = if (use_carry and self.getFlag(C_FLAG)) 1 else 0;
        const result = @as(u16, a) + @as(u16, b) + carry;
        self.setFlag(H_FLAG, ((a & 0x0F) + (b & 0x0F) + @as(u8, @intCast(carry))) > 0x0F);
        self.setFlag(C_FLAG, result > 0xFF);
        const final = @as(u8, @intCast(result & 0xFF));
        self.setFlag(Z_FLAG, final == 0);
        self.setFlag(N_FLAG, false);
        return final;
    }

    /// Generic 8-bit SUB/SBC with borrow support and return value
    inline fn sub8(self: *CPU, a: u8, b: u8, use_carry: bool) u8 {
        const carry: u8 = if (use_carry and self.getFlag(C_FLAG)) 1 else 0;
        self.setFlag(H_FLAG, (a & 0x0F) < (b & 0x0F) + carry);
        self.setFlag(C_FLAG, @as(u16, a) < @as(u16, b) + carry);
        const result = a -% b -% carry;
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, true);
        return result;
    }

    /// Generic 8-bit CP (compare) with flags - like SUB but doesn't store result
    inline fn cp8(self: *CPU, a: u8, b: u8) u8 {
        const result = a -% b;
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, true);
        self.setFlag(H_FLAG, (a & 0x0F) < (b & 0x0F));
        self.setFlag(C_FLAG, a < b);
        return result;
    }

    /// Generic relative jump helper
    inline fn relativeJump(self: *CPU, offset: i8) void {
        if (offset >= 0) {
            self.PC +%= @intCast(offset);
        } else {
            self.PC -%= @intCast(-offset);
        }
    }

    inline fn add_sp_e8_impl(self: *CPU, offset: i8) u16 {
        const sp = self.SP;

        // Interpret offset as 8-bit value, like the GB does
        const off_u8: u8 = @bitCast(offset); // signed -> u8, preserves bits
        const off_u16: u16 = @intCast(off_u8);

        const result: u16 = sp +% off_u16;

        self.setFlag(Z_FLAG, false);
        self.setFlag(N_FLAG, false);

        const sp_low: u8 = @truncate(sp);
        const sum16: u16 = @as(u16, sp_low) + @as(u16, off_u8);

        // Half-carry from bit 3
        self.setFlag(H_FLAG, ((sp_low & 0x0F) + (off_u8 & 0x0F)) > 0x0F);

        // Carry from bit 7 (byte)
        self.setFlag(C_FLAG, sum16 > 0xFF);

        return result;
    }

    // 0x00: NOP
    fn op_nop(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = self;
        _ = system_bus;
        return 4;
    }

    // 0x01: LD BC, n16
    fn op_ld_bc_n16(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try self.read16Immediate(system_bus);
        set16BitRegister(&self.B, &self.C, val);
        return 12;
    }

    // 0x02: LD [BC], A
    fn op_ld_bc_a(self: *CPU, system_bus: *SystemBus) !u32 {
        try system_bus.write(get16BitRegister(self.B, self.C), self.A);
        return 8;
    }

    // 0x03: INC BC
    fn op_inc_bc(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        inc16(&self.B, &self.C);
        return 8;
    }

    // 0x04: INC B
    fn op_inc_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.inc8(&self.B);
        return 4;
    }

    // 0x05: DEC B
    fn op_dec_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.dec8(&self.B);
        return 4;
    }

    // 0x06: LD B, n8
    fn op_ld_b_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        self.B = try system_bus.read(self.PC);
        self.PC +%= 1;
        return 8;
    }

    // 0x0C: INC C
    fn op_inc_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.inc8(&self.C);
        return 4;
    }

    // 0x0D: DEC C
    fn op_dec_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.dec8(&self.C);
        return 4;
    }

    // 0x0E: LD C, n8
    fn op_ld_c_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        self.C = try system_bus.read(self.PC);
        self.PC +%= 1;
        return 8;
    }

    // 0x11: LD DE, n16
    fn op_ld_de_n16(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try self.read16Immediate(system_bus);
        set16BitRegister(&self.D, &self.E, val);
        return 12;
    }

    // 0x13: INC DE
    fn op_inc_de(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        inc16(&self.D, &self.E);
        return 8;
    }

    // 0x18: JR e8
    fn op_jr_e8(self: *CPU, system_bus: *SystemBus) !u32 {
        const offset: i8 = @bitCast(try system_bus.read(self.PC));
        self.PC +%= 1;
        self.relativeJump(offset);
        return 12;
    }

    // 0x1E: LD E, n8
    fn op_ld_e_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        self.E = try system_bus.read(self.PC);
        self.PC +%= 1;
        return 8;
    }

    // 0x20: JR NZ, e8
    fn op_jr_nz_e8(self: *CPU, system_bus: *SystemBus) !u32 {
        const offset: i8 = @bitCast(try system_bus.read(self.PC));
        self.PC +%= 1;
        if (!self.getFlag(Z_FLAG)) {
            self.relativeJump(offset);
            return 12;
        }
        return 8;
    }

    // 0x21: LD HL, n16
    fn op_ld_hl_n16(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try self.read16Immediate(system_bus);
        set16BitRegister(&self.H, &self.L, val);
        return 12;
    }

    // 0x22: LD (HL+), A - LDI (HL), A
    fn op_ld_hli_a(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        try system_bus.write(addr, self.A);
        inc16(&self.H, &self.L);
        return 8;
    }

    // 0x23: INC HL
    fn op_inc_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        inc16(&self.H, &self.L);
        return 8;
    }

    // 0x28: JR Z, e8
    fn op_jr_z_e8(self: *CPU, system_bus: *SystemBus) !u32 {
        const offset: i8 = @bitCast(try system_bus.read(self.PC));
        self.PC +%= 1;
        if (self.getFlag(Z_FLAG)) {
            self.relativeJump(offset);
            return 12;
        }
        return 8;
    }

    // 0x2A: LD A, (HL+) - LDI A, (HL)
    fn op_ld_a_hli(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        self.A = try system_bus.read(addr);
        inc16(&self.H, &self.L);
        return 8;
    }

    // 0x2B: DEC HL
    fn op_dec_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        dec16(&self.H, &self.L);
        return 8;
    }

    // 0x2E: LD L, n8
    fn op_ld_l_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        self.L = try system_bus.read(self.PC);
        self.PC +%= 1;
        return 8;
    }

    // 0x31: LD SP, n16
    fn op_ld_sp_n16(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP = try self.read16Immediate(system_bus);
        return 12;
    }

    // 0x32: LD (HL-), A - LDD (HL), A
    fn op_ld_hld_a(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        try system_bus.write(addr, self.A);
        dec16(&self.H, &self.L);
        return 8;
    }

    // 0x36: LD (HL), n8
    fn op_ld_hl_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(self.PC);
        self.PC +%= 1;
        const addr = get16BitRegister(self.H, self.L);
        try system_bus.write(addr, val);
        return 12;
    }

    // 0x3A: LD A, (HL-) - LDD A, (HL)
    fn op_ld_a_hld(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        self.A = try system_bus.read(addr);
        dec16(&self.H, &self.L);
        return 8;
    }

    // 0x3C: INC A
    fn op_inc_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.inc8(&self.A);
        return 4;
    }

    // 0x3D: DEC A
    fn op_dec_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.dec8(&self.A);
        return 4;
    }

    // 0x3E: LD A, n8
    fn op_ld_a_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        self.A = try system_bus.read(self.PC);
        self.PC +%= 1;
        return 8;
    }

    // 0x76: HALT
    fn op_halt(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.halted = true;
        return 4;
    }

    // 0xC1: POP BC
    fn op_pop_bc(self: *CPU, system_bus: *SystemBus) !u32 {
        self.C = try system_bus.read(self.SP);
        self.SP +%= 1;
        self.B = try system_bus.read(self.SP);
        self.SP +%= 1;
        return 12;
    }

    // 0xC2: JP NZ, nn
    fn op_jp_nz_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        if (!self.getFlag(Z_FLAG)) {
            self.PC = addr;
            return 16;
        }
        return 12;
    }

    // 0xC3: JP nn
    fn op_jp_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        self.PC = try self.read16Immediate(system_bus);
        return 16;
    }

    // 0xC5: PUSH BC
    fn op_push_bc(self: *CPU, system_bus: *SystemBus) !u32 {
        const bc = get16BitRegister(self.B, self.C);
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast((bc >> 8) & 0xFF)); // high
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast(bc & 0xFF)); // low
        return 16;
    }

    // 0xC6: ADD A, n8
    fn op_add_a_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(self.PC);
        self.PC +%= 1;
        self.A = self.add8(self.A, val, false);
        return 8;
    }

    // 0xC9: RET
    fn op_ret(self: *CPU, system_bus: *SystemBus) !u32 {
        const low = try system_bus.read(self.SP);
        self.SP +%= 1;
        const high = try system_bus.read(self.SP);
        self.SP +%= 1;
        self.PC = get16BitRegister(high, low);
        return 16;
    }

    // 0xCA: JP Z, nn
    fn op_jp_z_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        if (self.getFlag(Z_FLAG)) {
            self.PC = addr;
            return 16;
        }
        return 12;
    }

    // 0xCD: CALL nn
    fn op_call_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);

        // Push return address onto stack
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast(self.PC & 0xFF));

        self.PC = addr;
        return 24;
    }

    // 0xFE: CP A, n8 (or CP n8)
    fn op_cp_a_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(self.PC);
        self.PC +%= 1;
        _ = self.cp8(self.A, val);
        return 8;
    }

    // ==================== MORE MISSING OPCODES ====================

    // 0x07: RLCA - Rotate A left
    fn op_rlca(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        const carry = (self.A & 0x80) != 0;
        self.A = (self.A << 1) | @as(u8, if (carry) 1 else 0);
        self.setFlag(Z_FLAG, false);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, carry);
        return 4;
    }

    // 0x0A: LD A, (BC)
    fn op_ld_a_bc(self: *CPU, system_bus: *SystemBus) !u32 {
        self.A = try system_bus.read(get16BitRegister(self.B, self.C));
        return 8;
    }

    // 0x0B: DEC BC
    fn op_dec_bc(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        dec16(&self.B, &self.C);
        return 8;
    }

    // 0x0F: RRCA - Rotate A right
    fn op_rrca(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        const carry = (self.A & 0x01) != 0;
        self.A = (self.A >> 1) | @as(u8, if (carry) 0x80 else 0);
        self.setFlag(Z_FLAG, false);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, carry);
        return 4;
    }

    // 0x10: STOP
    fn op_stop(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.halted = true; // Simplified - STOP is similar to HALT
        self.PC +%= 1; // STOP is 2 bytes
        return 4;
    }

    // 0x12: LD (DE), A
    fn op_ld_de_a(self: *CPU, system_bus: *SystemBus) !u32 {
        try system_bus.write(get16BitRegister(self.D, self.E), self.A);
        return 8;
    }

    // 0x14: INC D
    fn op_inc_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.inc8(&self.D);
        return 4;
    }

    // 0x15: DEC D
    fn op_dec_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.dec8(&self.D);
        return 4;
    }

    // 0x16: LD D, n8
    fn op_ld_d_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        self.D = try system_bus.read(self.PC);
        self.PC +%= 1;
        return 8;
    }

    // 0x17: RLA - Rotate A left through carry
    fn op_rla(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        const old_carry: u8 = if (self.getFlag(C_FLAG)) 1 else 0;
        const new_carry = (self.A & 0x80) != 0;
        self.A = (self.A << 1) | old_carry;
        self.setFlag(Z_FLAG, false);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, new_carry);
        return 4;
    }

    // 0x1A: LD A, (DE)
    fn op_ld_a_de(self: *CPU, system_bus: *SystemBus) !u32 {
        self.A = try system_bus.read(get16BitRegister(self.D, self.E));
        return 8;
    }

    // 0x1B: DEC DE
    fn op_dec_de(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        dec16(&self.D, &self.E);
        return 8;
    }

    // 0x1C: INC E
    fn op_inc_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.inc8(&self.E);
        return 4;
    }

    // 0x1D: DEC E
    fn op_dec_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.dec8(&self.E);
        return 4;
    }

    // 0x1F: RRA - Rotate A right through carry
    fn op_rra(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        const old_carry: u8 = if (self.getFlag(C_FLAG)) 0x80 else 0;
        const new_carry = (self.A & 0x01) != 0;
        self.A = (self.A >> 1) | old_carry;
        self.setFlag(Z_FLAG, false);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, new_carry);
        return 4;
    }

    // 0x24: INC H
    fn op_inc_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.inc8(&self.H);
        return 4;
    }

    // 0x25: DEC H
    fn op_dec_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.dec8(&self.H);
        return 4;
    }

    // 0x26: LD H, n8
    fn op_ld_h_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        self.H = try system_bus.read(self.PC);
        self.PC +%= 1;
        return 8;
    }

    // 0x27: DAA - Decimal Adjust Accumulator (for BCD)
    fn op_daa(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        var a = self.A;

        if (!self.getFlag(N_FLAG)) {
            // After addition
            if (self.getFlag(C_FLAG) or a > 0x99) {
                a +%= 0x60;
                self.setFlag(C_FLAG, true);
            }
            if (self.getFlag(H_FLAG) or (a & 0x0F) > 0x09) {
                a +%= 0x06;
            }
        } else {
            // After subtraction
            if (self.getFlag(C_FLAG)) {
                a -%= 0x60;
            }
            if (self.getFlag(H_FLAG)) {
                a -%= 0x06;
            }
        }

        self.A = a;
        self.setFlag(Z_FLAG, a == 0);
        self.setFlag(H_FLAG, false);
        return 4;
    }

    // 0x2C: INC L
    fn op_inc_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.inc8(&self.L);
        return 4;
    }

    // 0x2D: DEC L
    fn op_dec_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.dec8(&self.L);
        return 4;
    }

    // 0x2F: CPL - Complement A (flip all bits)
    fn op_cpl(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = ~self.A;
        self.setFlag(N_FLAG, true);
        self.setFlag(H_FLAG, true);
        return 4;
    }

    // 0x30: JR NC, e8
    fn op_jr_nc_e8(self: *CPU, system_bus: *SystemBus) !u32 {
        const offset: i8 = @bitCast(try system_bus.read(self.PC));
        self.PC +%= 1;
        if (!self.getFlag(C_FLAG)) {
            self.relativeJump(offset);
            return 12;
        }
        return 8;
    }

    // 0x33: INC SP
    fn op_inc_sp(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.SP +%= 1;
        return 8;
    }

    // 0x34: INC (HL)
    fn op_inc_hl_mem(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        var val = try system_bus.read(addr);
        const old = val;
        val +%= 1;
        try system_bus.write(addr, val);
        self.setFlag(Z_FLAG, val == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, (old & 0x0F) + 1 > 0x0F);
        return 12;
    }

    // 0x35: DEC (HL)
    fn op_dec_hl_mem(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        var val = try system_bus.read(addr);
        const old = val;
        val -%= 1;
        try system_bus.write(addr, val);
        self.setFlag(Z_FLAG, val == 0);
        self.setFlag(N_FLAG, true);
        self.setFlag(H_FLAG, (old & 0x0F) == 0x00);
        return 12;
    }

    // 0x37: SCF - Set Carry Flag
    fn op_scf(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, true);
        return 4;
    }

    // 0x38: JR C, e8
    fn op_jr_c_e8(self: *CPU, system_bus: *SystemBus) !u32 {
        const offset: i8 = @bitCast(try system_bus.read(self.PC));
        self.PC +%= 1;
        if (self.getFlag(C_FLAG)) {
            self.relativeJump(offset);
            return 12;
        }
        return 8;
    }

    // 0x3B: DEC SP
    fn op_dec_sp(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.SP -%= 1;
        return 8;
    }

    // 0x3F: CCF - Complement Carry Flag
    fn op_ccf(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, !self.getFlag(C_FLAG));
        return 4;
    }

    // ==================== 8-bit register LD block (0x40-0x7F) ====================
    // 0x40-0x47: LD B,r
    fn op_ld_b_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.B;
        return 4;
    }
    fn op_ld_b_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.C;
        return 4;
    }
    fn op_ld_b_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.D;
        return 4;
    }
    fn op_ld_b_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.E;
        return 4;
    }
    fn op_ld_b_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.H;
        return 4;
    }
    fn op_ld_b_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.L;
        return 4;
    }
    fn op_ld_b_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        self.B = try system_bus.read(get16BitRegister(self.H, self.L));
        return 8;
    }
    fn op_ld_b_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.A;
        return 4;
    }

    // 0x48-0x4F: LD C,r
    fn op_ld_c_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.B;
        return 4;
    }
    fn op_ld_c_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.C;
        return 4;
    }
    fn op_ld_c_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.D;
        return 4;
    }
    fn op_ld_c_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.E;
        return 4;
    }
    fn op_ld_c_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.H;
        return 4;
    }
    fn op_ld_c_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.L;
        return 4;
    }
    fn op_ld_c_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        self.C = try system_bus.read(get16BitRegister(self.H, self.L));
        return 8;
    }
    fn op_ld_c_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.A;
        return 4;
    }

    // 0x50-0x57: LD D,r
    fn op_ld_d_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.B;
        return 4;
    }
    fn op_ld_d_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.C;
        return 4;
    }
    fn op_ld_d_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.D;
        return 4;
    }
    fn op_ld_d_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.E;
        return 4;
    }
    fn op_ld_d_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.H;
        return 4;
    }
    fn op_ld_d_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.L;
        return 4;
    }
    fn op_ld_d_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        self.D = try system_bus.read(get16BitRegister(self.H, self.L));
        return 8;
    }
    fn op_ld_d_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.A;
        return 4;
    }

    // 0x58-0x5F: LD E,r
    fn op_ld_e_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.B;
        return 4;
    }
    fn op_ld_e_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.C;
        return 4;
    }
    fn op_ld_e_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.D;
        return 4;
    }
    fn op_ld_e_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.E;
        return 4;
    }
    fn op_ld_e_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.H;
        return 4;
    }
    fn op_ld_e_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.L;
        return 4;
    }
    fn op_ld_e_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        self.E = try system_bus.read(get16BitRegister(self.H, self.L));
        return 8;
    }
    fn op_ld_e_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.A;
        return 4;
    }

    // 0x60-0x67: LD H,r
    fn op_ld_h_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.B;
        return 4;
    }
    fn op_ld_h_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.C;
        return 4;
    }
    fn op_ld_h_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.D;
        return 4;
    }
    fn op_ld_h_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.E;
        return 4;
    }
    fn op_ld_h_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.H;
        return 4;
    }
    fn op_ld_h_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.L;
        return 4;
    }
    fn op_ld_h_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        self.H = try system_bus.read(get16BitRegister(self.H, self.L));
        return 8;
    }
    fn op_ld_h_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.A;
        return 4;
    }

    // 0x68-0x6F: LD L,r
    fn op_ld_l_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.B;
        return 4;
    }
    fn op_ld_l_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.C;
        return 4;
    }
    fn op_ld_l_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.D;
        return 4;
    }
    fn op_ld_l_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.E;
        return 4;
    }
    fn op_ld_l_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.H;
        return 4;
    }
    fn op_ld_l_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.L;
        return 4;
    }
    fn op_ld_l_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        self.L = try system_bus.read(get16BitRegister(self.H, self.L));
        return 8;
    }
    fn op_ld_l_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.A;
        return 4;
    }

    // 0x70-0x77: LD (HL),r
    fn op_ld_hl_b(self: *CPU, system_bus: *SystemBus) !u32 {
        try system_bus.write(get16BitRegister(self.H, self.L), self.B);
        return 8;
    }
    fn op_ld_hl_c(self: *CPU, system_bus: *SystemBus) !u32 {
        try system_bus.write(get16BitRegister(self.H, self.L), self.C);
        return 8;
    }
    fn op_ld_hl_d(self: *CPU, system_bus: *SystemBus) !u32 {
        try system_bus.write(get16BitRegister(self.H, self.L), self.D);
        return 8;
    }
    fn op_ld_hl_e(self: *CPU, system_bus: *SystemBus) !u32 {
        try system_bus.write(get16BitRegister(self.H, self.L), self.E);
        return 8;
    }
    fn op_ld_hl_h(self: *CPU, system_bus: *SystemBus) !u32 {
        try system_bus.write(get16BitRegister(self.H, self.L), self.H);
        return 8;
    }
    fn op_ld_hl_l(self: *CPU, system_bus: *SystemBus) !u32 {
        try system_bus.write(get16BitRegister(self.H, self.L), self.L);
        return 8;
    }
    // 0x76: HALT is already defined
    fn op_ld_hl_a(self: *CPU, system_bus: *SystemBus) !u32 {
        try system_bus.write(get16BitRegister(self.H, self.L), self.A);
        return 8;
    }

    // 0x78-0x7F: LD A,r
    fn op_ld_a_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.B;
        return 4;
    }
    fn op_ld_a_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.C;
        return 4;
    }
    fn op_ld_a_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.D;
        return 4;
    }
    fn op_ld_a_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.E;
        return 4;
    }
    fn op_ld_a_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.H;
        return 4;
    }
    fn op_ld_a_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.L;
        return 4;
    }
    fn op_ld_a_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        self.A = try system_bus.read(get16BitRegister(self.H, self.L));
        return 8;
    }
    fn op_ld_a_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.A;
        return 4;
    }

    // ==================== 8-bit arithmetic (0x80-0xBF) ====================
    // 0x80-0x87: ADD A,r
    fn op_add_a_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.B, false);
        return 4;
    }
    fn op_add_a_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.C, false);
        return 4;
    }
    fn op_add_a_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.D, false);
        return 4;
    }
    fn op_add_a_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.E, false);
        return 4;
    }
    fn op_add_a_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.H, false);
        return 4;
    }
    fn op_add_a_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.L, false);
        return 4;
    }
    fn op_add_a_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(get16BitRegister(self.H, self.L));
        self.A = self.add8(self.A, val, false);
        return 8;
    }
    fn op_add_a_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.A, false);
        return 4;
    }

    // 0x88-0x8F: ADC A,r
    fn op_adc_a_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.B, true);
        return 4;
    }
    fn op_adc_a_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.C, true);
        return 4;
    }
    fn op_adc_a_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.D, true);
        return 4;
    }
    fn op_adc_a_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.E, true);
        return 4;
    }
    fn op_adc_a_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.H, true);
        return 4;
    }
    fn op_adc_a_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.L, true);
        return 4;
    }
    fn op_adc_a_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(get16BitRegister(self.H, self.L));
        self.A = self.add8(self.A, val, true);
        return 8;
    }
    fn op_adc_a_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.add8(self.A, self.A, true);
        return 4;
    }

    // 0x90-0x97: SUB A,r
    fn op_sub_a_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.B, false);
        return 4;
    }
    fn op_sub_a_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.C, false);
        return 4;
    }
    fn op_sub_a_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.D, false);
        return 4;
    }
    fn op_sub_a_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.E, false);
        return 4;
    }
    fn op_sub_a_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.H, false);
        return 4;
    }
    fn op_sub_a_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.L, false);
        return 4;
    }
    fn op_sub_a_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(get16BitRegister(self.H, self.L));
        self.A = self.sub8(self.A, val, false);
        return 8;
    }
    fn op_sub_a_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.A, false);
        return 4;
    }

    // 0x98-0x9F: SBC A,r
    fn op_sbc_a_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.B, true);
        return 4;
    }
    fn op_sbc_a_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.C, true);
        return 4;
    }
    fn op_sbc_a_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.D, true);
        return 4;
    }
    fn op_sbc_a_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.E, true);
        return 4;
    }
    fn op_sbc_a_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.H, true);
        return 4;
    }
    fn op_sbc_a_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.L, true);
        return 4;
    }
    fn op_sbc_a_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(get16BitRegister(self.H, self.L));
        self.A = self.sub8(self.A, val, true);
        return 8;
    }
    fn op_sbc_a_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sub8(self.A, self.A, true);
        return 4;
    }

    // 0xA0-0xA7: AND A,r
    inline fn and8(self: *CPU, a: u8, b: u8) u8 {
        const result = a & b;
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, true);
        self.setFlag(C_FLAG, false);
        return result;
    }
    fn op_and_a_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.and8(self.A, self.B);
        return 4;
    }
    fn op_and_a_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.and8(self.A, self.C);
        return 4;
    }
    fn op_and_a_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.and8(self.A, self.D);
        return 4;
    }
    fn op_and_a_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.and8(self.A, self.E);
        return 4;
    }
    fn op_and_a_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.and8(self.A, self.H);
        return 4;
    }
    fn op_and_a_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.and8(self.A, self.L);
        return 4;
    }
    fn op_and_a_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(get16BitRegister(self.H, self.L));
        self.A = self.and8(self.A, val);
        return 8;
    }
    fn op_and_a_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.and8(self.A, self.A);
        return 4;
    }

    // 0xA8-0xAF: XOR A,r
    inline fn xor8(self: *CPU, a: u8, b: u8) u8 {
        const result = a ^ b;
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, false);
        return result;
    }
    fn op_xor_a_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.xor8(self.A, self.B);
        return 4;
    }
    fn op_xor_a_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.xor8(self.A, self.C);
        return 4;
    }
    fn op_xor_a_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.xor8(self.A, self.D);
        return 4;
    }
    fn op_xor_a_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.xor8(self.A, self.E);
        return 4;
    }
    fn op_xor_a_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.xor8(self.A, self.H);
        return 4;
    }
    fn op_xor_a_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.xor8(self.A, self.L);
        return 4;
    }
    fn op_xor_a_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(get16BitRegister(self.H, self.L));
        self.A = self.xor8(self.A, val);
        return 8;
    }
    fn op_xor_a_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.xor8(self.A, self.A);
        return 4;
    }
    // 0xAF: XOR A - Special case alias for clarity
    const op_xor_a = op_xor_a_a;

    // 0xB0-0xB7: OR A,r
    inline fn or8(self: *CPU, a: u8, b: u8) u8 {
        const result = a | b;
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, false);
        return result;
    }
    fn op_or_a_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.or8(self.A, self.B);
        return 4;
    }
    fn op_or_a_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.or8(self.A, self.C);
        return 4;
    }
    fn op_or_a_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.or8(self.A, self.D);
        return 4;
    }
    fn op_or_a_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.or8(self.A, self.E);
        return 4;
    }
    fn op_or_a_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.or8(self.A, self.H);
        return 4;
    }
    fn op_or_a_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.or8(self.A, self.L);
        return 4;
    }
    fn op_or_a_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(get16BitRegister(self.H, self.L));
        self.A = self.or8(self.A, val);
        return 8;
    }
    fn op_or_a_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.or8(self.A, self.A);
        return 4;
    }

    // 0xB8-0xBF: CP A,r
    fn op_cp_a_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        _ = self.cp8(self.A, self.B);
        return 4;
    }
    fn op_cp_a_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        _ = self.cp8(self.A, self.C);
        return 4;
    }
    fn op_cp_a_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        _ = self.cp8(self.A, self.D);
        return 4;
    }
    fn op_cp_a_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        _ = self.cp8(self.A, self.E);
        return 4;
    }
    fn op_cp_a_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        _ = self.cp8(self.A, self.H);
        return 4;
    }
    fn op_cp_a_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        _ = self.cp8(self.A, self.L);
        return 4;
    }
    fn op_cp_a_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(get16BitRegister(self.H, self.L));
        _ = self.cp8(self.A, val);
        return 8;
    }
    fn op_cp_a_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        _ = self.cp8(self.A, self.A);
        return 4;
    }

    // ==================== Remaining unprefixed opcodes ====================

    // 0x08: LD (nn),SP
    fn op_ld_nn_sp(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        try system_bus.write(addr, @intCast(self.SP & 0xFF));
        try system_bus.write(addr +% 1, @intCast((self.SP >> 8) & 0xFF));
        return 20;
    }

    // 0x09: ADD HL,BC
    fn op_add_hl_bc(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        const hl = get16BitRegister(self.H, self.L);
        const bc = get16BitRegister(self.B, self.C);
        const result = @as(u32, hl) + @as(u32, bc);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, ((hl & 0x0FFF) + (bc & 0x0FFF)) > 0x0FFF);
        self.setFlag(C_FLAG, result > 0xFFFF);
        set16BitRegister(&self.H, &self.L, @intCast(result & 0xFFFF));
        return 8;
    }

    // 0x19: ADD HL,DE
    fn op_add_hl_de(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        const hl = get16BitRegister(self.H, self.L);
        const de = get16BitRegister(self.D, self.E);
        const result = @as(u32, hl) + @as(u32, de);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, ((hl & 0x0FFF) + (de & 0x0FFF)) > 0x0FFF);
        self.setFlag(C_FLAG, result > 0xFFFF);
        set16BitRegister(&self.H, &self.L, @intCast(result & 0xFFFF));
        return 8;
    }

    // 0x29: ADD HL,HL
    fn op_add_hl_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        const hl = get16BitRegister(self.H, self.L);
        const result = @as(u32, hl) + @as(u32, hl);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, ((hl & 0x0FFF) + (hl & 0x0FFF)) > 0x0FFF);
        self.setFlag(C_FLAG, result > 0xFFFF);
        set16BitRegister(&self.H, &self.L, @intCast(result & 0xFFFF));
        return 8;
    }

    // 0x39: ADD HL,SP
    fn op_add_hl_sp(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        const hl = get16BitRegister(self.H, self.L);
        const result = @as(u32, hl) + @as(u32, self.SP);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, ((hl & 0x0FFF) + (self.SP & 0x0FFF)) > 0x0FFF);
        self.setFlag(C_FLAG, result > 0xFFFF);
        set16BitRegister(&self.H, &self.L, @intCast(result & 0xFFFF));
        return 8;
    }

    // 0xC0: RET NZ
    fn op_ret_nz(self: *CPU, system_bus: *SystemBus) !u32 {
        if (!self.getFlag(Z_FLAG)) {
            const low = try system_bus.read(self.SP);
            self.SP +%= 1;
            const high = try system_bus.read(self.SP);
            self.SP +%= 1;
            self.PC = get16BitRegister(high, low);
            return 20;
        }
        return 8;
    }

    // 0xC4: CALL NZ,nn
    fn op_call_nz_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        if (!self.getFlag(Z_FLAG)) {
            self.SP -%= 1;
            try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
            self.SP -%= 1;
            try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
            self.PC = addr;
            return 24;
        }
        return 12;
    }

    // 0xC7-0xFF: RST vectors and remaining ops
    fn op_rst_00(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
        self.PC = 0x0000;
        return 16;
    }
    fn op_rst_08(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
        self.PC = 0x0008;
        return 16;
    }
    fn op_rst_10(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
        self.PC = 0x0010;
        return 16;
    }
    fn op_rst_18(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
        self.PC = 0x0018;
        return 16;
    }
    fn op_rst_20(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
        self.PC = 0x0020;
        return 16;
    }
    fn op_rst_28(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
        self.PC = 0x0028;
        return 16;
    }
    fn op_rst_30(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
        self.PC = 0x0030;
        return 16;
    }
    fn op_rst_38(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
        self.SP -%= 1;
        try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
        self.PC = 0x0038;
        return 16;
    }

    // 0xC8: RET Z
    fn op_ret_z(self: *CPU, system_bus: *SystemBus) !u32 {
        if (self.getFlag(Z_FLAG)) {
            const low = try system_bus.read(self.SP);
            self.SP +%= 1;
            const high = try system_bus.read(self.SP);
            self.SP +%= 1;
            self.PC = get16BitRegister(high, low);
            return 20;
        }
        return 8;
    }

    // 0xCC: CALL Z,nn
    fn op_call_z_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        if (self.getFlag(Z_FLAG)) {
            self.SP -%= 1;
            try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
            self.SP -%= 1;
            try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
            self.PC = addr;
            return 24;
        }
        return 12;
    }

    // 0xCE: ADC A,n8
    fn op_adc_a_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(self.PC);
        self.PC +%= 1;
        self.A = self.add8(self.A, val, true);
        return 8;
    }

    // 0xD0: RET NC
    fn op_ret_nc(self: *CPU, system_bus: *SystemBus) !u32 {
        if (!self.getFlag(C_FLAG)) {
            const low = try system_bus.read(self.SP);
            self.SP +%= 1;
            const high = try system_bus.read(self.SP);
            self.SP +%= 1;
            self.PC = get16BitRegister(high, low);
            return 20;
        }
        return 8;
    }

    // 0xD1: POP DE
    fn op_pop_de(self: *CPU, system_bus: *SystemBus) !u32 {
        self.E = try system_bus.read(self.SP);
        self.SP +%= 1;
        self.D = try system_bus.read(self.SP);
        self.SP +%= 1;
        return 12;
    }

    // 0xD2: JP NC,nn
    fn op_jp_nc_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        if (!self.getFlag(C_FLAG)) {
            self.PC = addr;
            return 16;
        }
        return 12;
    }

    // 0xD4: CALL NC,nn
    fn op_call_nc_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        if (!self.getFlag(C_FLAG)) {
            self.SP -%= 1;
            try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
            self.SP -%= 1;
            try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
            self.PC = addr;
            return 24;
        }
        return 12;
    }

    // 0xD5: PUSH DE
    fn op_push_de(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, self.D);
        self.SP -%= 1;
        try system_bus.write(self.SP, self.E);
        return 16;
    }

    // 0xD6: SUB A,n8
    fn op_sub_a_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(self.PC);
        self.PC +%= 1;
        self.A = self.sub8(self.A, val, false);
        return 8;
    }

    // 0xD8: RET C
    fn op_ret_c(self: *CPU, system_bus: *SystemBus) !u32 {
        if (self.getFlag(C_FLAG)) {
            const low = try system_bus.read(self.SP);
            self.SP +%= 1;
            const high = try system_bus.read(self.SP);
            self.SP +%= 1;
            self.PC = get16BitRegister(high, low);
            return 20;
        }
        return 8;
    }

    // 0xD9: RETI - Return and enable interrupts
    fn op_reti(self: *CPU, system_bus: *SystemBus) !u32 {
        const low = try system_bus.read(self.SP);
        self.SP +%= 1;
        const high = try system_bus.read(self.SP);
        self.SP +%= 1;
        self.PC = get16BitRegister(high, low);
        self.ime = true; // Enable interrupts
        return 16;
    }

    // 0xDA: JP C,nn
    fn op_jp_c_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        if (self.getFlag(C_FLAG)) {
            self.PC = addr;
            return 16;
        }
        return 12;
    }

    // 0xDC: CALL C,nn
    fn op_call_c_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        if (self.getFlag(C_FLAG)) {
            self.SP -%= 1;
            try system_bus.write(self.SP, @intCast((self.PC >> 8) & 0xFF));
            self.SP -%= 1;
            try system_bus.write(self.SP, @intCast(self.PC & 0xFF));
            self.PC = addr;
            return 24;
        }
        return 12;
    }

    // 0xDE: SBC A,n8
    fn op_sbc_a_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(self.PC);
        self.PC +%= 1;
        self.A = self.sub8(self.A, val, true);
        return 8;
    }

    // 0xE0: LDH (n8),A
    fn op_ldh_n8_a(self: *CPU, system_bus: *SystemBus) !u32 {
        const offset = try system_bus.read(self.PC);
        self.PC +%= 1;
        try system_bus.write(0xFF00 + @as(u16, offset), self.A);
        return 12;
    }

    // 0xE1: POP HL
    fn op_pop_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        self.L = try system_bus.read(self.SP);
        self.SP +%= 1;
        self.H = try system_bus.read(self.SP);
        self.SP +%= 1;
        return 12;
    }

    // 0xE2: LD (C),A
    fn op_ld_c_mem_a(self: *CPU, system_bus: *SystemBus) !u32 {
        try system_bus.write(0xFF00 + @as(u16, self.C), self.A);
        return 8;
    }

    // 0xE5: PUSH HL
    fn op_push_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, self.H);
        self.SP -%= 1;
        try system_bus.write(self.SP, self.L);
        return 16;
    }

    // 0xE6: AND A,n8
    fn op_and_a_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(self.PC);
        self.PC +%= 1;
        self.A = self.and8(self.A, val);
        return 8;
    }

    // 0xE8: ADD SP,e8
    fn op_add_sp_e8(self: *CPU, system_bus: *SystemBus) !u32 {
        const offset: i8 = @bitCast(try system_bus.read(self.PC));
        self.PC +%= 1;
        self.SP = self.add_sp_e8_impl(offset);
        return 16;
    }

    // 0xE9: JP HL
    fn op_jp_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.PC = get16BitRegister(self.H, self.L);
        return 4;
    }

    // 0xEA: LD (nn),A
    fn op_ld_nn_a(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        try system_bus.write(addr, self.A);
        return 16;
    }

    // 0xEE: XOR A,n8
    fn op_xor_a_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(self.PC);
        self.PC +%= 1;
        self.A = self.xor8(self.A, val);
        return 8;
    }

    // 0xF0: LDH A,(n8)
    fn op_ldh_a_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        const offset = try system_bus.read(self.PC);
        self.PC +%= 1;
        self.A = try system_bus.read(0xFF00 + @as(u16, offset));
        return 12;
    }

    // 0xF1: POP AF
    fn op_pop_af(self: *CPU, system_bus: *SystemBus) !u32 {
        self.F = try system_bus.read(self.SP);
        self.SP +%= 1;
        self.A = try system_bus.read(self.SP);
        self.SP +%= 1;
        // Mask F register to only keep flag bits
        self.F &= 0xF0;
        return 12;
    }

    // 0xF2: LD A,(C)
    fn op_ld_a_c_mem(self: *CPU, system_bus: *SystemBus) !u32 {
        self.A = try system_bus.read(0xFF00 + @as(u16, self.C));
        return 8;
    }

    // 0xF3: DI - Disable interrupts
    fn op_di(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.ime = false;
        return 4;
    }

    // 0xF5: PUSH AF
    fn op_push_af(self: *CPU, system_bus: *SystemBus) !u32 {
        self.SP -%= 1;
        try system_bus.write(self.SP, self.A);
        self.SP -%= 1;
        try system_bus.write(self.SP, self.F);
        return 16;
    }

    // 0xF6: OR A,n8
    fn op_or_a_n8(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(self.PC);
        self.PC +%= 1;
        self.A = self.or8(self.A, val);
        return 8;
    }

    // 0xF8: LD HL,SP+e8
    fn op_ld_hl_sp_e8(self: *CPU, system_bus: *SystemBus) !u32 {
        const offset: i8 = @bitCast(try system_bus.read(self.PC));
        self.PC +%= 1;
        const result = self.add_sp_e8_impl(offset);
        set16BitRegister(&self.H, &self.L, result);
        return 12;
    }

    // 0xF9: LD SP,HL
    fn op_ld_sp_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.SP = get16BitRegister(self.H, self.L);
        return 8;
    }

    // 0xFA: LD A,(nn)
    fn op_ld_a_nn(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = try self.read16Immediate(system_bus);
        self.A = try system_bus.read(addr);
        return 16;
    }

    // 0xFB: EI - Enable interrupts
    fn op_ei(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.ime = true;
        return 4;
    }

    // Unimplemented opcode handler
    fn op_unimplemented(self: *CPU, system_bus: *SystemBus) !u32 {
        const val = try system_bus.read(self.PC - 1);
        std.debug.print("Unimplemented Opcode is: {d}, called @ {d}\n", .{ val, self.PC });
        return error.UnimplementedOpcode;
    }

    // ==================== CB-Prefixed Instructions ====================

    // Helper functions for CB instructions
    inline fn rlc(self: *CPU, val: u8) u8 {
        const carry = (val & 0x80) != 0;
        const result = (val << 1) | @as(u8, if (carry) 1 else 0);
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, carry);
        return result;
    }

    inline fn rrc(self: *CPU, val: u8) u8 {
        const carry = (val & 0x01) != 0;
        const result = (val >> 1) | @as(u8, if (carry) 0x80 else 0);
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, carry);
        return result;
    }

    inline fn rl(self: *CPU, val: u8) u8 {
        const old_carry: u8 = if (self.getFlag(C_FLAG)) 1 else 0;
        const new_carry = (val & 0x80) != 0;
        const result = (val << 1) | old_carry;
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, new_carry);
        return result;
    }

    inline fn rr(self: *CPU, val: u8) u8 {
        const old_carry: u8 = if (self.getFlag(C_FLAG)) 0x80 else 0;
        const new_carry = (val & 0x01) != 0;
        const result = (val >> 1) | old_carry;
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, new_carry);
        return result;
    }

    inline fn sla(self: *CPU, val: u8) u8 {
        const carry = (val & 0x80) != 0;
        const result = val << 1;
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, carry);
        return result;
    }

    inline fn sra(self: *CPU, val: u8) u8 {
        const carry = (val & 0x01) != 0;
        const result = (val >> 1) | (val & 0x80);
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, carry);
        return result;
    }

    inline fn swap(self: *CPU, val: u8) u8 {
        const result = ((val & 0x0F) << 4) | ((val & 0xF0) >> 4);
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, false);
        return result;
    }

    inline fn srl(self: *CPU, val: u8) u8 {
        const carry = (val & 0x01) != 0;
        const result = val >> 1;
        self.setFlag(Z_FLAG, result == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, false);
        self.setFlag(C_FLAG, carry);
        return result;
    }

    inline fn bit(self: *CPU, val: u8, bit_pos: u3) void {
        const bit_val = (val >> bit_pos) & 1;
        self.setFlag(Z_FLAG, bit_val == 0);
        self.setFlag(N_FLAG, false);
        self.setFlag(H_FLAG, true);
    }

    // CB 0x00-0x07: RLC r
    fn cb_rlc_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.rlc(self.B);
        return 8;
    }
    fn cb_rlc_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.rlc(self.C);
        return 8;
    }
    fn cb_rlc_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.rlc(self.D);
        return 8;
    }
    fn cb_rlc_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.rlc(self.E);
        return 8;
    }
    fn cb_rlc_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.rlc(self.H);
        return 8;
    }
    fn cb_rlc_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.rlc(self.L);
        return 8;
    }
    fn cb_rlc_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, self.rlc(val));
        return 16;
    }
    fn cb_rlc_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.rlc(self.A);
        return 8;
    }

    // CB 0x08-0x0F: RRC r
    fn cb_rrc_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.rrc(self.B);
        return 8;
    }
    fn cb_rrc_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.rrc(self.C);
        return 8;
    }
    fn cb_rrc_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.rrc(self.D);
        return 8;
    }
    fn cb_rrc_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.rrc(self.E);
        return 8;
    }
    fn cb_rrc_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.rrc(self.H);
        return 8;
    }
    fn cb_rrc_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.rrc(self.L);
        return 8;
    }
    fn cb_rrc_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, self.rrc(val));
        return 16;
    }
    fn cb_rrc_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.rrc(self.A);
        return 8;
    }

    // CB 0x10-0x17: RL r
    fn cb_rl_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.rl(self.B);
        return 8;
    }
    fn cb_rl_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.rl(self.C);
        return 8;
    }
    fn cb_rl_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.rl(self.D);
        return 8;
    }
    fn cb_rl_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.rl(self.E);
        return 8;
    }
    fn cb_rl_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.rl(self.H);
        return 8;
    }
    fn cb_rl_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.rl(self.L);
        return 8;
    }
    fn cb_rl_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, self.rl(val));
        return 16;
    }
    fn cb_rl_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.rl(self.A);
        return 8;
    }

    // CB 0x18-0x1F: RR r
    fn cb_rr_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.rr(self.B);
        return 8;
    }
    fn cb_rr_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.rr(self.C);
        return 8;
    }
    fn cb_rr_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.rr(self.D);
        return 8;
    }
    fn cb_rr_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.rr(self.E);
        return 8;
    }
    fn cb_rr_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.rr(self.H);
        return 8;
    }
    fn cb_rr_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.rr(self.L);
        return 8;
    }
    fn cb_rr_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, self.rr(val));
        return 16;
    }
    fn cb_rr_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.rr(self.A);
        return 8;
    }

    // CB 0x20-0x27: SLA r
    fn cb_sla_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.sla(self.B);
        return 8;
    }
    fn cb_sla_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.sla(self.C);
        return 8;
    }
    fn cb_sla_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.sla(self.D);
        return 8;
    }
    fn cb_sla_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.sla(self.E);
        return 8;
    }
    fn cb_sla_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.sla(self.H);
        return 8;
    }
    fn cb_sla_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.sla(self.L);
        return 8;
    }
    fn cb_sla_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, self.sla(val));
        return 16;
    }
    fn cb_sla_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sla(self.A);
        return 8;
    }

    // CB 0x28-0x2F: SRA r
    fn cb_sra_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.sra(self.B);
        return 8;
    }
    fn cb_sra_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.sra(self.C);
        return 8;
    }
    fn cb_sra_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.sra(self.D);
        return 8;
    }
    fn cb_sra_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.sra(self.E);
        return 8;
    }
    fn cb_sra_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.sra(self.H);
        return 8;
    }
    fn cb_sra_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.sra(self.L);
        return 8;
    }
    fn cb_sra_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, self.sra(val));
        return 16;
    }
    fn cb_sra_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.sra(self.A);
        return 8;
    }

    // CB 0x30-0x37: SWAP r
    fn cb_swap_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.swap(self.B);
        return 8;
    }
    fn cb_swap_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.swap(self.C);
        return 8;
    }
    fn cb_swap_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.swap(self.D);
        return 8;
    }
    fn cb_swap_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.swap(self.E);
        return 8;
    }
    fn cb_swap_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.swap(self.H);
        return 8;
    }
    fn cb_swap_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.swap(self.L);
        return 8;
    }
    fn cb_swap_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, self.swap(val));
        return 16;
    }
    fn cb_swap_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.swap(self.A);
        return 8;
    }

    // CB 0x38-0x3F: SRL r
    fn cb_srl_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B = self.srl(self.B);
        return 8;
    }
    fn cb_srl_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C = self.srl(self.C);
        return 8;
    }
    fn cb_srl_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D = self.srl(self.D);
        return 8;
    }
    fn cb_srl_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E = self.srl(self.E);
        return 8;
    }
    fn cb_srl_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H = self.srl(self.H);
        return 8;
    }
    fn cb_srl_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L = self.srl(self.L);
        return 8;
    }
    fn cb_srl_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, self.srl(val));
        return 16;
    }
    fn cb_srl_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A = self.srl(self.A);
        return 8;
    }

    // CB 0x40-0x47: BIT 0, r
    fn cb_bit_0_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.B, 0);
        return 8;
    }
    fn cb_bit_0_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.C, 0);
        return 8;
    }
    fn cb_bit_0_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.D, 0);
        return 8;
    }
    fn cb_bit_0_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.E, 0);
        return 8;
    }
    fn cb_bit_0_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.H, 0);
        return 8;
    }
    fn cb_bit_0_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.L, 0);
        return 8;
    }
    fn cb_bit_0_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        self.bit(val, 0);
        return 12;
    }
    fn cb_bit_0_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.A, 0);
        return 8;
    }

    // CB 0x48-0x4F: BIT 1, r
    fn cb_bit_1_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.B, 1);
        return 8;
    }
    fn cb_bit_1_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.C, 1);
        return 8;
    }
    fn cb_bit_1_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.D, 1);
        return 8;
    }
    fn cb_bit_1_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.E, 1);
        return 8;
    }
    fn cb_bit_1_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.H, 1);
        return 8;
    }
    fn cb_bit_1_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.L, 1);
        return 8;
    }
    fn cb_bit_1_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        self.bit(val, 1);
        return 12;
    }
    fn cb_bit_1_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.A, 1);
        return 8;
    }

    // CB 0x50-0x57: BIT 2, r
    fn cb_bit_2_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.B, 2);
        return 8;
    }
    fn cb_bit_2_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.C, 2);
        return 8;
    }
    fn cb_bit_2_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.D, 2);
        return 8;
    }
    fn cb_bit_2_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.E, 2);
        return 8;
    }
    fn cb_bit_2_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.H, 2);
        return 8;
    }
    fn cb_bit_2_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.L, 2);
        return 8;
    }
    fn cb_bit_2_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        self.bit(val, 2);
        return 12;
    }
    fn cb_bit_2_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.A, 2);
        return 8;
    }

    // CB 0x58-0x5F: BIT 3, r
    fn cb_bit_3_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.B, 3);
        return 8;
    }
    fn cb_bit_3_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.C, 3);
        return 8;
    }
    fn cb_bit_3_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.D, 3);
        return 8;
    }
    fn cb_bit_3_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.E, 3);
        return 8;
    }
    fn cb_bit_3_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.H, 3);
        return 8;
    }
    fn cb_bit_3_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.L, 3);
        return 8;
    }
    fn cb_bit_3_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        self.bit(val, 3);
        return 12;
    }
    fn cb_bit_3_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.A, 3);
        return 8;
    }

    // CB 0x60-0x67: BIT 4, r
    fn cb_bit_4_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.B, 4);
        return 8;
    }
    fn cb_bit_4_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.C, 4);
        return 8;
    }
    fn cb_bit_4_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.D, 4);
        return 8;
    }
    fn cb_bit_4_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.E, 4);
        return 8;
    }
    fn cb_bit_4_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.H, 4);
        return 8;
    }
    fn cb_bit_4_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.L, 4);
        return 8;
    }
    fn cb_bit_4_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        self.bit(val, 4);
        return 12;
    }
    fn cb_bit_4_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.A, 4);
        return 8;
    }

    // CB 0x68-0x6F: BIT 5, r
    fn cb_bit_5_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.B, 5);
        return 8;
    }
    fn cb_bit_5_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.C, 5);
        return 8;
    }
    fn cb_bit_5_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.D, 5);
        return 8;
    }
    fn cb_bit_5_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.E, 5);
        return 8;
    }
    fn cb_bit_5_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.H, 5);
        return 8;
    }
    fn cb_bit_5_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.L, 5);
        return 8;
    }
    fn cb_bit_5_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        self.bit(val, 5);
        return 12;
    }
    fn cb_bit_5_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.A, 5);
        return 8;
    }

    // CB 0x70-0x77: BIT 6, r
    fn cb_bit_6_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.B, 6);
        return 8;
    }
    fn cb_bit_6_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.C, 6);
        return 8;
    }
    fn cb_bit_6_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.D, 6);
        return 8;
    }
    fn cb_bit_6_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.E, 6);
        return 8;
    }
    fn cb_bit_6_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.H, 6);
        return 8;
    }
    fn cb_bit_6_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.L, 6);
        return 8;
    }
    fn cb_bit_6_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        self.bit(val, 6);
        return 12;
    }
    fn cb_bit_6_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.A, 6);
        return 8;
    }

    // CB 0x78-0x7F: BIT 7, r
    fn cb_bit_7_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.B, 7);
        return 8;
    }
    fn cb_bit_7_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.C, 7);
        return 8;
    }
    fn cb_bit_7_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.D, 7);
        return 8;
    }
    fn cb_bit_7_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.E, 7);
        return 8;
    }
    fn cb_bit_7_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.H, 7);
        return 8;
    }
    fn cb_bit_7_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.L, 7);
        return 8;
    }
    fn cb_bit_7_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        self.bit(val, 7);
        return 12;
    }
    fn cb_bit_7_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.bit(self.A, 7);
        return 8;
    }

    // CB 0x80-0x87: RES 0, r
    fn cb_res_0_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B &= ~(@as(u8, 1) << 0);
        return 8;
    }
    fn cb_res_0_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C &= ~(@as(u8, 1) << 0);
        return 8;
    }
    fn cb_res_0_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D &= ~(@as(u8, 1) << 0);
        return 8;
    }
    fn cb_res_0_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E &= ~(@as(u8, 1) << 0);
        return 8;
    }
    fn cb_res_0_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H &= ~(@as(u8, 1) << 0);
        return 8;
    }
    fn cb_res_0_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L &= ~(@as(u8, 1) << 0);
        return 8;
    }
    fn cb_res_0_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val & ~(@as(u8, 1) << 0));
        return 16;
    }
    fn cb_res_0_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A &= ~(@as(u8, 1) << 0);
        return 8;
    }

    // CB 0x88-0x8F: RES 1, r
    fn cb_res_1_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B &= ~(@as(u8, 1) << 1);
        return 8;
    }
    fn cb_res_1_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C &= ~(@as(u8, 1) << 1);
        return 8;
    }
    fn cb_res_1_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D &= ~(@as(u8, 1) << 1);
        return 8;
    }
    fn cb_res_1_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E &= ~(@as(u8, 1) << 1);
        return 8;
    }
    fn cb_res_1_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H &= ~(@as(u8, 1) << 1);
        return 8;
    }
    fn cb_res_1_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L &= ~(@as(u8, 1) << 1);
        return 8;
    }
    fn cb_res_1_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val & ~(@as(u8, 1) << 1));
        return 16;
    }
    fn cb_res_1_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A &= ~(@as(u8, 1) << 1);
        return 8;
    }

    // CB 0x90-0x97: RES 2, r
    fn cb_res_2_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B &= ~(@as(u8, 1) << 2);
        return 8;
    }
    fn cb_res_2_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C &= ~(@as(u8, 1) << 2);
        return 8;
    }
    fn cb_res_2_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D &= ~(@as(u8, 1) << 2);
        return 8;
    }
    fn cb_res_2_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E &= ~(@as(u8, 1) << 2);
        return 8;
    }
    fn cb_res_2_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H &= ~(@as(u8, 1) << 2);
        return 8;
    }
    fn cb_res_2_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L &= ~(@as(u8, 1) << 2);
        return 8;
    }
    fn cb_res_2_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val & ~(@as(u8, 1) << 2));
        return 16;
    }
    fn cb_res_2_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A &= ~(@as(u8, 1) << 2);
        return 8;
    }

    // CB 0x98-0x9F: RES 3, r
    fn cb_res_3_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B &= ~(@as(u8, 1) << 3);
        return 8;
    }
    fn cb_res_3_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C &= ~(@as(u8, 1) << 3);
        return 8;
    }
    fn cb_res_3_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D &= ~(@as(u8, 1) << 3);
        return 8;
    }
    fn cb_res_3_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E &= ~(@as(u8, 1) << 3);
        return 8;
    }
    fn cb_res_3_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H &= ~(@as(u8, 1) << 3);
        return 8;
    }
    fn cb_res_3_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L &= ~(@as(u8, 1) << 3);
        return 8;
    }
    fn cb_res_3_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val & ~(@as(u8, 1) << 3));
        return 16;
    }
    fn cb_res_3_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A &= ~(@as(u8, 1) << 3);
        return 8;
    }

    // CB 0xA0-0xA7: RES 4, r
    fn cb_res_4_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B &= ~(@as(u8, 1) << 4);
        return 8;
    }
    fn cb_res_4_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C &= ~(@as(u8, 1) << 4);
        return 8;
    }
    fn cb_res_4_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D &= ~(@as(u8, 1) << 4);
        return 8;
    }
    fn cb_res_4_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E &= ~(@as(u8, 1) << 4);
        return 8;
    }
    fn cb_res_4_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H &= ~(@as(u8, 1) << 4);
        return 8;
    }
    fn cb_res_4_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L &= ~(@as(u8, 1) << 4);
        return 8;
    }
    fn cb_res_4_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val & ~(@as(u8, 1) << 4));
        return 16;
    }
    fn cb_res_4_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A &= ~(@as(u8, 1) << 4);
        return 8;
    }

    // CB 0xA8-0xAF: RES 5, r
    fn cb_res_5_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B &= ~(@as(u8, 1) << 5);
        return 8;
    }
    fn cb_res_5_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C &= ~(@as(u8, 1) << 5);
        return 8;
    }
    fn cb_res_5_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D &= ~(@as(u8, 1) << 5);
        return 8;
    }
    fn cb_res_5_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E &= ~(@as(u8, 1) << 5);
        return 8;
    }
    fn cb_res_5_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H &= ~(@as(u8, 1) << 5);
        return 8;
    }
    fn cb_res_5_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L &= ~(@as(u8, 1) << 5);
        return 8;
    }
    fn cb_res_5_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val & ~(@as(u8, 1) << 5));
        return 16;
    }
    fn cb_res_5_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A &= ~(@as(u8, 1) << 5);
        return 8;
    }

    // CB 0xB0-0xB7: RES 6, r
    fn cb_res_6_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B &= ~(@as(u8, 1) << 6);
        return 8;
    }
    fn cb_res_6_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C &= ~(@as(u8, 1) << 6);
        return 8;
    }
    fn cb_res_6_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D &= ~(@as(u8, 1) << 6);
        return 8;
    }
    fn cb_res_6_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E &= ~(@as(u8, 1) << 6);
        return 8;
    }
    fn cb_res_6_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H &= ~(@as(u8, 1) << 6);
        return 8;
    }
    fn cb_res_6_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L &= ~(@as(u8, 1) << 6);
        return 8;
    }
    fn cb_res_6_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val & ~(@as(u8, 1) << 6));
        return 16;
    }
    fn cb_res_6_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A &= ~(@as(u8, 1) << 6);
        return 8;
    }

    // CB 0xB8-0xBF: RES 7, r
    fn cb_res_7_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B &= ~(@as(u8, 1) << 7);
        return 8;
    }
    fn cb_res_7_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C &= ~(@as(u8, 1) << 7);
        return 8;
    }
    fn cb_res_7_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D &= ~(@as(u8, 1) << 7);
        return 8;
    }
    fn cb_res_7_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E &= ~(@as(u8, 1) << 7);
        return 8;
    }
    fn cb_res_7_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H &= ~(@as(u8, 1) << 7);
        return 8;
    }
    fn cb_res_7_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L &= ~(@as(u8, 1) << 7);
        return 8;
    }
    fn cb_res_7_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val & ~(@as(u8, 1) << 7));
        return 16;
    }
    fn cb_res_7_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A &= ~(@as(u8, 1) << 7);
        return 8;
    }

    // CB 0xC0-0xC7: SET 0, r
    fn cb_set_0_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B |= (@as(u8, 1) << 0);
        return 8;
    }
    fn cb_set_0_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C |= (@as(u8, 1) << 0);
        return 8;
    }
    fn cb_set_0_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D |= (@as(u8, 1) << 0);
        return 8;
    }
    fn cb_set_0_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E |= (@as(u8, 1) << 0);
        return 8;
    }
    fn cb_set_0_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H |= (@as(u8, 1) << 0);
        return 8;
    }
    fn cb_set_0_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L |= (@as(u8, 1) << 0);
        return 8;
    }
    fn cb_set_0_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val | (@as(u8, 1) << 0));
        return 16;
    }
    fn cb_set_0_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A |= (@as(u8, 1) << 0);
        return 8;
    }

    // CB 0xC8-0xCF: SET 1, r
    fn cb_set_1_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B |= (@as(u8, 1) << 1);
        return 8;
    }
    fn cb_set_1_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C |= (@as(u8, 1) << 1);
        return 8;
    }
    fn cb_set_1_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D |= (@as(u8, 1) << 1);
        return 8;
    }
    fn cb_set_1_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E |= (@as(u8, 1) << 1);
        return 8;
    }
    fn cb_set_1_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H |= (@as(u8, 1) << 1);
        return 8;
    }
    fn cb_set_1_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L |= (@as(u8, 1) << 1);
        return 8;
    }
    fn cb_set_1_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val | (@as(u8, 1) << 1));
        return 16;
    }
    fn cb_set_1_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A |= (@as(u8, 1) << 1);
        return 8;
    }

    // CB 0xD0-0xD7: SET 2, r
    fn cb_set_2_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B |= (@as(u8, 1) << 2);
        return 8;
    }
    fn cb_set_2_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C |= (@as(u8, 1) << 2);
        return 8;
    }
    fn cb_set_2_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D |= (@as(u8, 1) << 2);
        return 8;
    }
    fn cb_set_2_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E |= (@as(u8, 1) << 2);
        return 8;
    }
    fn cb_set_2_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H |= (@as(u8, 1) << 2);
        return 8;
    }
    fn cb_set_2_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L |= (@as(u8, 1) << 2);
        return 8;
    }
    fn cb_set_2_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val | (@as(u8, 1) << 2));
        return 16;
    }
    fn cb_set_2_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A |= (@as(u8, 1) << 2);
        return 8;
    }

    // CB 0xD8-0xDF: SET 3, r
    fn cb_set_3_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B |= (@as(u8, 1) << 3);
        return 8;
    }
    fn cb_set_3_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C |= (@as(u8, 1) << 3);
        return 8;
    }
    fn cb_set_3_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D |= (@as(u8, 1) << 3);
        return 8;
    }
    fn cb_set_3_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E |= (@as(u8, 1) << 3);
        return 8;
    }
    fn cb_set_3_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H |= (@as(u8, 1) << 3);
        return 8;
    }
    fn cb_set_3_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L |= (@as(u8, 1) << 3);
        return 8;
    }
    fn cb_set_3_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val | (@as(u8, 1) << 3));
        return 16;
    }
    fn cb_set_3_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A |= (@as(u8, 1) << 3);
        return 8;
    }

    // CB 0xE0-0xE7: SET 4, r
    fn cb_set_4_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B |= (@as(u8, 1) << 4);
        return 8;
    }
    fn cb_set_4_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C |= (@as(u8, 1) << 4);
        return 8;
    }
    fn cb_set_4_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D |= (@as(u8, 1) << 4);
        return 8;
    }
    fn cb_set_4_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E |= (@as(u8, 1) << 4);
        return 8;
    }
    fn cb_set_4_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H |= (@as(u8, 1) << 4);
        return 8;
    }
    fn cb_set_4_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L |= (@as(u8, 1) << 4);
        return 8;
    }
    fn cb_set_4_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val | (@as(u8, 1) << 4));
        return 16;
    }
    fn cb_set_4_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A |= (@as(u8, 1) << 4);
        return 8;
    }

    // CB 0xE8-0xEF: SET 5, r
    fn cb_set_5_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B |= (@as(u8, 1) << 5);
        return 8;
    }
    fn cb_set_5_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C |= (@as(u8, 1) << 5);
        return 8;
    }
    fn cb_set_5_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D |= (@as(u8, 1) << 5);
        return 8;
    }
    fn cb_set_5_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E |= (@as(u8, 1) << 5);
        return 8;
    }
    fn cb_set_5_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H |= (@as(u8, 1) << 5);
        return 8;
    }
    fn cb_set_5_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L |= (@as(u8, 1) << 5);
        return 8;
    }
    fn cb_set_5_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val | (@as(u8, 1) << 5));
        return 16;
    }
    fn cb_set_5_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A |= (@as(u8, 1) << 5);
        return 8;
    }

    // CB 0xF0-0xF7: SET 6, r
    fn cb_set_6_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B |= (@as(u8, 1) << 6);
        return 8;
    }
    fn cb_set_6_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C |= (@as(u8, 1) << 6);
        return 8;
    }
    fn cb_set_6_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D |= (@as(u8, 1) << 6);
        return 8;
    }
    fn cb_set_6_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E |= (@as(u8, 1) << 6);
        return 8;
    }
    fn cb_set_6_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H |= (@as(u8, 1) << 6);
        return 8;
    }
    fn cb_set_6_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L |= (@as(u8, 1) << 6);
        return 8;
    }
    fn cb_set_6_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val | (@as(u8, 1) << 6));
        return 16;
    }
    fn cb_set_6_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A |= (@as(u8, 1) << 6);
        return 8;
    }

    // CB 0xF8-0xFF: SET 7, r
    fn cb_set_7_b(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.B |= (@as(u8, 1) << 7);
        return 8;
    }
    fn cb_set_7_c(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.C |= (@as(u8, 1) << 7);
        return 8;
    }
    fn cb_set_7_d(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.D |= (@as(u8, 1) << 7);
        return 8;
    }
    fn cb_set_7_e(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.E |= (@as(u8, 1) << 7);
        return 8;
    }
    fn cb_set_7_h(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.H |= (@as(u8, 1) << 7);
        return 8;
    }
    fn cb_set_7_l(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.L |= (@as(u8, 1) << 7);
        return 8;
    }
    fn cb_set_7_hl(self: *CPU, system_bus: *SystemBus) !u32 {
        const addr = get16BitRegister(self.H, self.L);
        const val = try system_bus.read(addr);
        try system_bus.write(addr, val | (@as(u8, 1) << 7));
        return 16;
    }
    fn cb_set_7_a(self: *CPU, system_bus: *SystemBus) !u32 {
        _ = system_bus;
        self.A |= (@as(u8, 1) << 7);
        return 8;
    }

    // ==================== Opcode Lookup Table ====================

    var OPCODE_TABLE = init: {
        var table: [256]Instruction = undefined;

        // Initialize all opcodes as unimplemented
        for (&table) |*entry| {
            entry.* = .{
                .handler = op_unimplemented,
                .mnemonic = "UNIMPLEMENTED",
            };
        }

        // Define implemented opcodes
        table[0x00] = .{ .handler = op_nop, .mnemonic = "NOP" };
        table[0x01] = .{ .handler = op_ld_bc_n16, .mnemonic = "LD BC,n16" };
        table[0x02] = .{ .handler = op_ld_bc_a, .mnemonic = "LD [BC],A" };
        table[0x03] = .{ .handler = op_inc_bc, .mnemonic = "INC BC" };
        table[0x04] = .{ .handler = op_inc_b, .mnemonic = "INC B" };
        table[0x05] = .{ .handler = op_dec_b, .mnemonic = "DEC B" };
        table[0x06] = .{ .handler = op_ld_b_n8, .mnemonic = "LD B,n8" };
        table[0x07] = .{ .handler = op_rlca, .mnemonic = "RLCA" };
        table[0x08] = .{ .handler = op_ld_nn_sp, .mnemonic = "LD (nn),SP" };
        table[0x09] = .{ .handler = op_add_hl_bc, .mnemonic = "ADD HL,BC" };
        table[0x0A] = .{ .handler = op_ld_a_bc, .mnemonic = "LD A,(BC)" };
        table[0x0B] = .{ .handler = op_dec_bc, .mnemonic = "DEC BC" };
        table[0x0C] = .{ .handler = op_inc_c, .mnemonic = "INC C" };
        table[0x0D] = .{ .handler = op_dec_c, .mnemonic = "DEC C" };
        table[0x0E] = .{ .handler = op_ld_c_n8, .mnemonic = "LD C,n8" };
        table[0x0F] = .{ .handler = op_rrca, .mnemonic = "RRCA" };

        table[0x10] = .{ .handler = op_stop, .mnemonic = "STOP" };
        table[0x11] = .{ .handler = op_ld_de_n16, .mnemonic = "LD DE,n16" };
        table[0x12] = .{ .handler = op_ld_de_a, .mnemonic = "LD (DE),A" };
        table[0x13] = .{ .handler = op_inc_de, .mnemonic = "INC DE" };
        table[0x14] = .{ .handler = op_inc_d, .mnemonic = "INC D" };
        table[0x15] = .{ .handler = op_dec_d, .mnemonic = "DEC D" };
        table[0x16] = .{ .handler = op_ld_d_n8, .mnemonic = "LD D,n8" };
        table[0x17] = .{ .handler = op_rla, .mnemonic = "RLA" };
        table[0x18] = .{ .handler = op_jr_e8, .mnemonic = "JR e8" };
        table[0x19] = .{ .handler = op_add_hl_de, .mnemonic = "ADD HL,DE" };
        table[0x1A] = .{ .handler = op_ld_a_de, .mnemonic = "LD A,(DE)" };
        table[0x1B] = .{ .handler = op_dec_de, .mnemonic = "DEC DE" };
        table[0x1C] = .{ .handler = op_inc_e, .mnemonic = "INC E" };
        table[0x1D] = .{ .handler = op_dec_e, .mnemonic = "DEC E" };
        table[0x1E] = .{ .handler = op_ld_e_n8, .mnemonic = "LD E,n8" };
        table[0x1F] = .{ .handler = op_rra, .mnemonic = "RRA" };

        table[0x20] = .{ .handler = op_jr_nz_e8, .mnemonic = "JR NZ,e8" };
        table[0x21] = .{ .handler = op_ld_hl_n16, .mnemonic = "LD HL,n16" };
        table[0x22] = .{ .handler = op_ld_hli_a, .mnemonic = "LD (HL+),A" };
        table[0x23] = .{ .handler = op_inc_hl, .mnemonic = "INC HL" };
        table[0x24] = .{ .handler = op_inc_h, .mnemonic = "INC H" };
        table[0x25] = .{ .handler = op_dec_h, .mnemonic = "DEC H" };
        table[0x26] = .{ .handler = op_ld_h_n8, .mnemonic = "LD H,n8" };
        table[0x27] = .{ .handler = op_daa, .mnemonic = "DAA" };
        table[0x28] = .{ .handler = op_jr_z_e8, .mnemonic = "JR Z,e8" };
        table[0x29] = .{ .handler = op_add_hl_hl, .mnemonic = "ADD HL,HL" };
        table[0x2A] = .{ .handler = op_ld_a_hli, .mnemonic = "LD A,(HL+)" };
        table[0x2B] = .{ .handler = op_dec_hl, .mnemonic = "DEC HL" };
        table[0x2C] = .{ .handler = op_inc_l, .mnemonic = "INC L" };
        table[0x2D] = .{ .handler = op_dec_l, .mnemonic = "DEC L" };
        table[0x2E] = .{ .handler = op_ld_l_n8, .mnemonic = "LD L,n8" };
        table[0x2F] = .{ .handler = op_cpl, .mnemonic = "CPL" };

        table[0x30] = .{ .handler = op_jr_nc_e8, .mnemonic = "JR NC,e8" };
        table[0x31] = .{ .handler = op_ld_sp_n16, .mnemonic = "LD SP,n16" };
        table[0x32] = .{ .handler = op_ld_hld_a, .mnemonic = "LD (HL-),A" };
        table[0x33] = .{ .handler = op_inc_sp, .mnemonic = "INC SP" };
        table[0x34] = .{ .handler = op_inc_hl_mem, .mnemonic = "INC (HL)" };
        table[0x35] = .{ .handler = op_dec_hl_mem, .mnemonic = "DEC (HL)" };
        table[0x36] = .{ .handler = op_ld_hl_n8, .mnemonic = "LD (HL),n8" };
        table[0x37] = .{ .handler = op_scf, .mnemonic = "SCF" };
        table[0x38] = .{ .handler = op_jr_c_e8, .mnemonic = "JR C,e8" };
        table[0x39] = .{ .handler = op_add_hl_sp, .mnemonic = "ADD HL,SP" };
        table[0x3A] = .{ .handler = op_ld_a_hld, .mnemonic = "LD A,(HL-)" };
        table[0x3B] = .{ .handler = op_dec_sp, .mnemonic = "DEC SP" };
        table[0x3C] = .{ .handler = op_inc_a, .mnemonic = "INC A" };
        table[0x3D] = .{ .handler = op_dec_a, .mnemonic = "DEC A" };
        table[0x3E] = .{ .handler = op_ld_a_n8, .mnemonic = "LD A,n8" };
        table[0x3F] = .{ .handler = op_ccf, .mnemonic = "CCF" };

        // 0x40-0x7F: 8-bit register LD instructions
        table[0x40] = .{ .handler = op_ld_b_b, .mnemonic = "LD B,B" };
        table[0x41] = .{ .handler = op_ld_b_c, .mnemonic = "LD B,C" };
        table[0x42] = .{ .handler = op_ld_b_d, .mnemonic = "LD B,D" };
        table[0x43] = .{ .handler = op_ld_b_e, .mnemonic = "LD B,E" };
        table[0x44] = .{ .handler = op_ld_b_h, .mnemonic = "LD B,H" };
        table[0x45] = .{ .handler = op_ld_b_l, .mnemonic = "LD B,L" };
        table[0x46] = .{ .handler = op_ld_b_hl, .mnemonic = "LD B,(HL)" };
        table[0x47] = .{ .handler = op_ld_b_a, .mnemonic = "LD B,A" };
        table[0x48] = .{ .handler = op_ld_c_b, .mnemonic = "LD C,B" };
        table[0x49] = .{ .handler = op_ld_c_c, .mnemonic = "LD C,C" };
        table[0x4A] = .{ .handler = op_ld_c_d, .mnemonic = "LD C,D" };
        table[0x4B] = .{ .handler = op_ld_c_e, .mnemonic = "LD C,E" };
        table[0x4C] = .{ .handler = op_ld_c_h, .mnemonic = "LD C,H" };
        table[0x4D] = .{ .handler = op_ld_c_l, .mnemonic = "LD C,L" };
        table[0x4E] = .{ .handler = op_ld_c_hl, .mnemonic = "LD C,(HL)" };
        table[0x4F] = .{ .handler = op_ld_c_a, .mnemonic = "LD C,A" };

        table[0x50] = .{ .handler = op_ld_d_b, .mnemonic = "LD D,B" };
        table[0x51] = .{ .handler = op_ld_d_c, .mnemonic = "LD D,C" };
        table[0x52] = .{ .handler = op_ld_d_d, .mnemonic = "LD D,D" };
        table[0x53] = .{ .handler = op_ld_d_e, .mnemonic = "LD D,E" };
        table[0x54] = .{ .handler = op_ld_d_h, .mnemonic = "LD D,H" };
        table[0x55] = .{ .handler = op_ld_d_l, .mnemonic = "LD D,L" };
        table[0x56] = .{ .handler = op_ld_d_hl, .mnemonic = "LD D,(HL)" };
        table[0x57] = .{ .handler = op_ld_d_a, .mnemonic = "LD D,A" };
        table[0x58] = .{ .handler = op_ld_e_b, .mnemonic = "LD E,B" };
        table[0x59] = .{ .handler = op_ld_e_c, .mnemonic = "LD E,C" };
        table[0x5A] = .{ .handler = op_ld_e_d, .mnemonic = "LD E,D" };
        table[0x5B] = .{ .handler = op_ld_e_e, .mnemonic = "LD E,E" };
        table[0x5C] = .{ .handler = op_ld_e_h, .mnemonic = "LD E,H" };
        table[0x5D] = .{ .handler = op_ld_e_l, .mnemonic = "LD E,L" };
        table[0x5E] = .{ .handler = op_ld_e_hl, .mnemonic = "LD E,(HL)" };
        table[0x5F] = .{ .handler = op_ld_e_a, .mnemonic = "LD E,A" };

        table[0x60] = .{ .handler = op_ld_h_b, .mnemonic = "LD H,B" };
        table[0x61] = .{ .handler = op_ld_h_c, .mnemonic = "LD H,C" };
        table[0x62] = .{ .handler = op_ld_h_d, .mnemonic = "LD H,D" };
        table[0x63] = .{ .handler = op_ld_h_e, .mnemonic = "LD H,E" };
        table[0x64] = .{ .handler = op_ld_h_h, .mnemonic = "LD H,H" };
        table[0x65] = .{ .handler = op_ld_h_l, .mnemonic = "LD H,L" };
        table[0x66] = .{ .handler = op_ld_h_hl, .mnemonic = "LD H,(HL)" };
        table[0x67] = .{ .handler = op_ld_h_a, .mnemonic = "LD H,A" };
        table[0x68] = .{ .handler = op_ld_l_b, .mnemonic = "LD L,B" };
        table[0x69] = .{ .handler = op_ld_l_c, .mnemonic = "LD L,C" };
        table[0x6A] = .{ .handler = op_ld_l_d, .mnemonic = "LD L,D" };
        table[0x6B] = .{ .handler = op_ld_l_e, .mnemonic = "LD L,E" };
        table[0x6C] = .{ .handler = op_ld_l_h, .mnemonic = "LD L,H" };
        table[0x6D] = .{ .handler = op_ld_l_l, .mnemonic = "LD L,L" };
        table[0x6E] = .{ .handler = op_ld_l_hl, .mnemonic = "LD L,(HL)" };
        table[0x6F] = .{ .handler = op_ld_l_a, .mnemonic = "LD L,A" };

        table[0x70] = .{ .handler = op_ld_hl_b, .mnemonic = "LD (HL),B" };
        table[0x71] = .{ .handler = op_ld_hl_c, .mnemonic = "LD (HL),C" };
        table[0x72] = .{ .handler = op_ld_hl_d, .mnemonic = "LD (HL),D" };
        table[0x73] = .{ .handler = op_ld_hl_e, .mnemonic = "LD (HL),E" };
        table[0x74] = .{ .handler = op_ld_hl_h, .mnemonic = "LD (HL),H" };
        table[0x75] = .{ .handler = op_ld_hl_l, .mnemonic = "LD (HL),L" };
        table[0x76] = .{ .handler = op_halt, .mnemonic = "HALT" };
        table[0x77] = .{ .handler = op_ld_hl_a, .mnemonic = "LD (HL),A" };
        table[0x78] = .{ .handler = op_ld_a_b, .mnemonic = "LD A,B" };
        table[0x79] = .{ .handler = op_ld_a_c, .mnemonic = "LD A,C" };
        table[0x7A] = .{ .handler = op_ld_a_d, .mnemonic = "LD A,D" };
        table[0x7B] = .{ .handler = op_ld_a_e, .mnemonic = "LD A,E" };
        table[0x7C] = .{ .handler = op_ld_a_h, .mnemonic = "LD A,H" };
        table[0x7D] = .{ .handler = op_ld_a_l, .mnemonic = "LD A,L" };
        table[0x7E] = .{ .handler = op_ld_a_hl, .mnemonic = "LD A,(HL)" };
        table[0x7F] = .{ .handler = op_ld_a_a, .mnemonic = "LD A,A" };

        // 0x80-0xBF: 8-bit arithmetic and logical operations
        table[0x80] = .{ .handler = op_add_a_b, .mnemonic = "ADD A,B" };
        table[0x81] = .{ .handler = op_add_a_c, .mnemonic = "ADD A,C" };
        table[0x82] = .{ .handler = op_add_a_d, .mnemonic = "ADD A,D" };
        table[0x83] = .{ .handler = op_add_a_e, .mnemonic = "ADD A,E" };
        table[0x84] = .{ .handler = op_add_a_h, .mnemonic = "ADD A,H" };
        table[0x85] = .{ .handler = op_add_a_l, .mnemonic = "ADD A,L" };
        table[0x86] = .{ .handler = op_add_a_hl, .mnemonic = "ADD A,(HL)" };
        table[0x87] = .{ .handler = op_add_a_a, .mnemonic = "ADD A,A" };
        table[0x88] = .{ .handler = op_adc_a_b, .mnemonic = "ADC A,B" };
        table[0x89] = .{ .handler = op_adc_a_c, .mnemonic = "ADC A,C" };
        table[0x8A] = .{ .handler = op_adc_a_d, .mnemonic = "ADC A,D" };
        table[0x8B] = .{ .handler = op_adc_a_e, .mnemonic = "ADC A,E" };
        table[0x8C] = .{ .handler = op_adc_a_h, .mnemonic = "ADC A,H" };
        table[0x8D] = .{ .handler = op_adc_a_l, .mnemonic = "ADC A,L" };
        table[0x8E] = .{ .handler = op_adc_a_hl, .mnemonic = "ADC A,(HL)" };
        table[0x8F] = .{ .handler = op_adc_a_a, .mnemonic = "ADC A,A" };

        table[0x90] = .{ .handler = op_sub_a_b, .mnemonic = "SUB B" };
        table[0x91] = .{ .handler = op_sub_a_c, .mnemonic = "SUB C" };
        table[0x92] = .{ .handler = op_sub_a_d, .mnemonic = "SUB D" };
        table[0x93] = .{ .handler = op_sub_a_e, .mnemonic = "SUB E" };
        table[0x94] = .{ .handler = op_sub_a_h, .mnemonic = "SUB H" };
        table[0x95] = .{ .handler = op_sub_a_l, .mnemonic = "SUB L" };
        table[0x96] = .{ .handler = op_sub_a_hl, .mnemonic = "SUB (HL)" };
        table[0x97] = .{ .handler = op_sub_a_a, .mnemonic = "SUB A" };
        table[0x98] = .{ .handler = op_sbc_a_b, .mnemonic = "SBC A,B" };
        table[0x99] = .{ .handler = op_sbc_a_c, .mnemonic = "SBC A,C" };
        table[0x9A] = .{ .handler = op_sbc_a_d, .mnemonic = "SBC A,D" };
        table[0x9B] = .{ .handler = op_sbc_a_e, .mnemonic = "SBC A,E" };
        table[0x9C] = .{ .handler = op_sbc_a_h, .mnemonic = "SBC A,H" };
        table[0x9D] = .{ .handler = op_sbc_a_l, .mnemonic = "SBC A,L" };
        table[0x9E] = .{ .handler = op_sbc_a_hl, .mnemonic = "SBC A,(HL)" };
        table[0x9F] = .{ .handler = op_sbc_a_a, .mnemonic = "SBC A,A" };

        table[0xA0] = .{ .handler = op_and_a_b, .mnemonic = "AND B" };
        table[0xA1] = .{ .handler = op_and_a_c, .mnemonic = "AND C" };
        table[0xA2] = .{ .handler = op_and_a_d, .mnemonic = "AND D" };
        table[0xA3] = .{ .handler = op_and_a_e, .mnemonic = "AND E" };
        table[0xA4] = .{ .handler = op_and_a_h, .mnemonic = "AND H" };
        table[0xA5] = .{ .handler = op_and_a_l, .mnemonic = "AND L" };
        table[0xA6] = .{ .handler = op_and_a_hl, .mnemonic = "AND (HL)" };
        table[0xA7] = .{ .handler = op_and_a_a, .mnemonic = "AND A" };
        table[0xA8] = .{ .handler = op_xor_a_b, .mnemonic = "XOR B" };
        table[0xA9] = .{ .handler = op_xor_a_c, .mnemonic = "XOR C" };
        table[0xAA] = .{ .handler = op_xor_a_d, .mnemonic = "XOR D" };
        table[0xAB] = .{ .handler = op_xor_a_e, .mnemonic = "XOR E" };
        table[0xAC] = .{ .handler = op_xor_a_h, .mnemonic = "XOR H" };
        table[0xAD] = .{ .handler = op_xor_a_l, .mnemonic = "XOR L" };
        table[0xAE] = .{ .handler = op_xor_a_hl, .mnemonic = "XOR (HL)" };
        table[0xAF] = .{ .handler = op_xor_a, .mnemonic = "XOR A" };

        table[0xB0] = .{ .handler = op_or_a_b, .mnemonic = "OR B" };
        table[0xB1] = .{ .handler = op_or_a_c, .mnemonic = "OR C" };
        table[0xB2] = .{ .handler = op_or_a_d, .mnemonic = "OR D" };
        table[0xB3] = .{ .handler = op_or_a_e, .mnemonic = "OR E" };
        table[0xB4] = .{ .handler = op_or_a_h, .mnemonic = "OR H" };
        table[0xB5] = .{ .handler = op_or_a_l, .mnemonic = "OR L" };
        table[0xB6] = .{ .handler = op_or_a_hl, .mnemonic = "OR (HL)" };
        table[0xB7] = .{ .handler = op_or_a_a, .mnemonic = "OR A" };
        table[0xB8] = .{ .handler = op_cp_a_b, .mnemonic = "CP B" };
        table[0xB9] = .{ .handler = op_cp_a_c, .mnemonic = "CP C" };
        table[0xBA] = .{ .handler = op_cp_a_d, .mnemonic = "CP D" };
        table[0xBB] = .{ .handler = op_cp_a_e, .mnemonic = "CP E" };
        table[0xBC] = .{ .handler = op_cp_a_h, .mnemonic = "CP H" };
        table[0xBD] = .{ .handler = op_cp_a_l, .mnemonic = "CP L" };
        table[0xBE] = .{ .handler = op_cp_a_hl, .mnemonic = "CP (HL)" };
        table[0xBF] = .{ .handler = op_cp_a_a, .mnemonic = "CP A" };

        table[0xC0] = .{ .handler = op_ret_nz, .mnemonic = "RET NZ" };
        table[0xC1] = .{ .handler = op_pop_bc, .mnemonic = "POP BC" };
        table[0xC2] = .{ .handler = op_jp_nz_nn, .mnemonic = "JP NZ,nn" };
        table[0xC3] = .{ .handler = op_jp_nn, .mnemonic = "JP nn" };
        table[0xC4] = .{ .handler = op_call_nz_nn, .mnemonic = "CALL NZ,nn" };
        table[0xC5] = .{ .handler = op_push_bc, .mnemonic = "PUSH BC" };
        table[0xC6] = .{ .handler = op_add_a_n8, .mnemonic = "ADD A,n8" };
        table[0xC7] = .{ .handler = op_rst_00, .mnemonic = "RST 00H" };
        table[0xC8] = .{ .handler = op_ret_z, .mnemonic = "RET Z" };
        table[0xC9] = .{ .handler = op_ret, .mnemonic = "RET" };
        table[0xCA] = .{ .handler = op_jp_z_nn, .mnemonic = "JP Z,nn" };
        // 0xCB is prefix for extended instructions
        table[0xCB] = .{ .handler = op_jp_c_nn, .mnemonic = "CB" };
        table[0xCC] = .{ .handler = op_call_z_nn, .mnemonic = "CALL Z,nn" };
        table[0xCD] = .{ .handler = op_call_nn, .mnemonic = "CALL nn" };
        table[0xCE] = .{ .handler = op_adc_a_n8, .mnemonic = "ADC A,n8" };
        table[0xCF] = .{ .handler = op_rst_08, .mnemonic = "RST 08H" };

        table[0xD0] = .{ .handler = op_ret_nc, .mnemonic = "RET NC" };
        table[0xD1] = .{ .handler = op_pop_de, .mnemonic = "POP DE" };
        table[0xD2] = .{ .handler = op_jp_nc_nn, .mnemonic = "JP NC,nn" };
        // 0xD3 is undefined
        table[0xD4] = .{ .handler = op_call_nc_nn, .mnemonic = "CALL NC,nn" };
        table[0xD5] = .{ .handler = op_push_de, .mnemonic = "PUSH DE" };
        table[0xD6] = .{ .handler = op_sub_a_n8, .mnemonic = "SUB n8" };
        table[0xD7] = .{ .handler = op_rst_10, .mnemonic = "RST 10H" };
        table[0xD8] = .{ .handler = op_ret_c, .mnemonic = "RET C" };
        table[0xD9] = .{ .handler = op_reti, .mnemonic = "RETI" };
        table[0xDA] = .{ .handler = op_jp_c_nn, .mnemonic = "JP C,nn" };
        // 0xDB is undefined
        table[0xDC] = .{ .handler = op_call_c_nn, .mnemonic = "CALL C,nn" };
        // 0xDD is undefined
        table[0xDE] = .{ .handler = op_sbc_a_n8, .mnemonic = "SBC A,n8" };
        table[0xDF] = .{ .handler = op_rst_18, .mnemonic = "RST 18H" };

        table[0xE0] = .{ .handler = op_ldh_n8_a, .mnemonic = "LDH (n8),A" };
        table[0xE1] = .{ .handler = op_pop_hl, .mnemonic = "POP HL" };
        table[0xE2] = .{ .handler = op_ld_c_mem_a, .mnemonic = "LD (C),A" };
        // 0xE3, 0xE4 are undefined
        table[0xE5] = .{ .handler = op_push_hl, .mnemonic = "PUSH HL" };
        table[0xE6] = .{ .handler = op_and_a_n8, .mnemonic = "AND n8" };
        table[0xE7] = .{ .handler = op_rst_20, .mnemonic = "RST 20H" };
        table[0xE8] = .{ .handler = op_add_sp_e8, .mnemonic = "ADD SP,e8" };
        table[0xE9] = .{ .handler = op_jp_hl, .mnemonic = "JP HL" };
        table[0xEA] = .{ .handler = op_ld_nn_a, .mnemonic = "LD (nn),A" };
        // 0xEB, 0xEC, 0xED are undefined
        table[0xEE] = .{ .handler = op_xor_a_n8, .mnemonic = "XOR n8" };
        table[0xEF] = .{ .handler = op_rst_28, .mnemonic = "RST 28H" };

        table[0xF0] = .{ .handler = op_ldh_a_n8, .mnemonic = "LDH A,(n8)" };
        table[0xF1] = .{ .handler = op_pop_af, .mnemonic = "POP AF" };
        table[0xF2] = .{ .handler = op_ld_a_c_mem, .mnemonic = "LD A,(C)" };
        table[0xF3] = .{ .handler = op_di, .mnemonic = "DI" };
        // 0xF4 is undefined
        table[0xF5] = .{ .handler = op_push_af, .mnemonic = "PUSH AF" };
        table[0xF6] = .{ .handler = op_or_a_n8, .mnemonic = "OR n8" };
        table[0xF7] = .{ .handler = op_rst_30, .mnemonic = "RST 30H" };
        table[0xF8] = .{ .handler = op_ld_hl_sp_e8, .mnemonic = "LD HL,SP+e8" };
        table[0xF9] = .{ .handler = op_ld_sp_hl, .mnemonic = "LD SP,HL" };
        table[0xFA] = .{ .handler = op_ld_a_nn, .mnemonic = "LD A,(nn)" };
        table[0xFB] = .{ .handler = op_ei, .mnemonic = "EI" };
        // 0xFC, 0xFD are undefined
        table[0xFE] = .{ .handler = op_cp_a_n8, .mnemonic = "CP n8" };
        table[0xFF] = .{ .handler = op_rst_38, .mnemonic = "RST 38H" };

        break :init table;
    };

    // CB-Prefixed Opcode Table (256 entries for all CB instructions)
    var CB_OPCODE_TABLE = init: {
        var table: [256]Instruction = undefined;

        // CB 0x00-0x07: RLC r
        table[0x00] = .{ .handler = cb_rlc_b, .mnemonic = "RLC B" };
        table[0x01] = .{ .handler = cb_rlc_c, .mnemonic = "RLC C" };
        table[0x02] = .{ .handler = cb_rlc_d, .mnemonic = "RLC D" };
        table[0x03] = .{ .handler = cb_rlc_e, .mnemonic = "RLC E" };
        table[0x04] = .{ .handler = cb_rlc_h, .mnemonic = "RLC H" };
        table[0x05] = .{ .handler = cb_rlc_l, .mnemonic = "RLC L" };
        table[0x06] = .{ .handler = cb_rlc_hl, .mnemonic = "RLC (HL)" };
        table[0x07] = .{ .handler = cb_rlc_a, .mnemonic = "RLC A" };

        // CB 0x08-0x0F: RRC r
        table[0x08] = .{ .handler = cb_rrc_b, .mnemonic = "RRC B" };
        table[0x09] = .{ .handler = cb_rrc_c, .mnemonic = "RRC C" };
        table[0x0A] = .{ .handler = cb_rrc_d, .mnemonic = "RRC D" };
        table[0x0B] = .{ .handler = cb_rrc_e, .mnemonic = "RRC E" };
        table[0x0C] = .{ .handler = cb_rrc_h, .mnemonic = "RRC H" };
        table[0x0D] = .{ .handler = cb_rrc_l, .mnemonic = "RRC L" };
        table[0x0E] = .{ .handler = cb_rrc_hl, .mnemonic = "RRC (HL)" };
        table[0x0F] = .{ .handler = cb_rrc_a, .mnemonic = "RRC A" };

        // CB 0x10-0x17: RL r
        table[0x10] = .{ .handler = cb_rl_b, .mnemonic = "RL B" };
        table[0x11] = .{ .handler = cb_rl_c, .mnemonic = "RL C" };
        table[0x12] = .{ .handler = cb_rl_d, .mnemonic = "RL D" };
        table[0x13] = .{ .handler = cb_rl_e, .mnemonic = "RL E" };
        table[0x14] = .{ .handler = cb_rl_h, .mnemonic = "RL H" };
        table[0x15] = .{ .handler = cb_rl_l, .mnemonic = "RL L" };
        table[0x16] = .{ .handler = cb_rl_hl, .mnemonic = "RL (HL)" };
        table[0x17] = .{ .handler = cb_rl_a, .mnemonic = "RL A" };

        // CB 0x18-0x1F: RR r
        table[0x18] = .{ .handler = cb_rr_b, .mnemonic = "RR B" };
        table[0x19] = .{ .handler = cb_rr_c, .mnemonic = "RR C" };
        table[0x1A] = .{ .handler = cb_rr_d, .mnemonic = "RR D" };
        table[0x1B] = .{ .handler = cb_rr_e, .mnemonic = "RR E" };
        table[0x1C] = .{ .handler = cb_rr_h, .mnemonic = "RR H" };
        table[0x1D] = .{ .handler = cb_rr_l, .mnemonic = "RR L" };
        table[0x1E] = .{ .handler = cb_rr_hl, .mnemonic = "RR (HL)" };
        table[0x1F] = .{ .handler = cb_rr_a, .mnemonic = "RR A" };

        // CB 0x20-0x27: SLA r
        table[0x20] = .{ .handler = cb_sla_b, .mnemonic = "SLA B" };
        table[0x21] = .{ .handler = cb_sla_c, .mnemonic = "SLA C" };
        table[0x22] = .{ .handler = cb_sla_d, .mnemonic = "SLA D" };
        table[0x23] = .{ .handler = cb_sla_e, .mnemonic = "SLA E" };
        table[0x24] = .{ .handler = cb_sla_h, .mnemonic = "SLA H" };
        table[0x25] = .{ .handler = cb_sla_l, .mnemonic = "SLA L" };
        table[0x26] = .{ .handler = cb_sla_hl, .mnemonic = "SLA (HL)" };
        table[0x27] = .{ .handler = cb_sla_a, .mnemonic = "SLA A" };

        // CB 0x28-0x2F: SRA r
        table[0x28] = .{ .handler = cb_sra_b, .mnemonic = "SRA B" };
        table[0x29] = .{ .handler = cb_sra_c, .mnemonic = "SRA C" };
        table[0x2A] = .{ .handler = cb_sra_d, .mnemonic = "SRA D" };
        table[0x2B] = .{ .handler = cb_sra_e, .mnemonic = "SRA E" };
        table[0x2C] = .{ .handler = cb_sra_h, .mnemonic = "SRA H" };
        table[0x2D] = .{ .handler = cb_sra_l, .mnemonic = "SRA L" };
        table[0x2E] = .{ .handler = cb_sra_hl, .mnemonic = "SRA (HL)" };
        table[0x2F] = .{ .handler = cb_sra_a, .mnemonic = "SRA A" };

        // CB 0x30-0x31: SWAP r (first 2)
        table[0x30] = .{ .handler = cb_swap_b, .mnemonic = "SWAP B" };
        table[0x31] = .{ .handler = cb_swap_c, .mnemonic = "SWAP C" };

        // CB 0x32-0x37: SWAP r (continued)
        table[0x32] = .{ .handler = cb_swap_d, .mnemonic = "SWAP D" };
        table[0x33] = .{ .handler = cb_swap_e, .mnemonic = "SWAP E" };
        table[0x34] = .{ .handler = cb_swap_h, .mnemonic = "SWAP H" };
        table[0x35] = .{ .handler = cb_swap_l, .mnemonic = "SWAP L" };
        table[0x36] = .{ .handler = cb_swap_hl, .mnemonic = "SWAP (HL)" };
        table[0x37] = .{ .handler = cb_swap_a, .mnemonic = "SWAP A" };

        // CB 0x38-0x3F: SRL r
        table[0x38] = .{ .handler = cb_srl_b, .mnemonic = "SRL B" };
        table[0x39] = .{ .handler = cb_srl_c, .mnemonic = "SRL C" };
        table[0x3A] = .{ .handler = cb_srl_d, .mnemonic = "SRL D" };
        table[0x3B] = .{ .handler = cb_srl_e, .mnemonic = "SRL E" };
        table[0x3C] = .{ .handler = cb_srl_h, .mnemonic = "SRL H" };
        table[0x3D] = .{ .handler = cb_srl_l, .mnemonic = "SRL L" };
        table[0x3E] = .{ .handler = cb_srl_hl, .mnemonic = "SRL (HL)" };
        table[0x3F] = .{ .handler = cb_srl_a, .mnemonic = "SRL A" };

        // CB 0x40-0x47: BIT 0, r
        table[0x40] = .{ .handler = cb_bit_0_b, .mnemonic = "BIT 0,B" };
        table[0x41] = .{ .handler = cb_bit_0_c, .mnemonic = "BIT 0,C" };
        table[0x42] = .{ .handler = cb_bit_0_d, .mnemonic = "BIT 0,D" };
        table[0x43] = .{ .handler = cb_bit_0_e, .mnemonic = "BIT 0,E" };
        table[0x44] = .{ .handler = cb_bit_0_h, .mnemonic = "BIT 0,H" };
        table[0x45] = .{ .handler = cb_bit_0_l, .mnemonic = "BIT 0,L" };
        table[0x46] = .{ .handler = cb_bit_0_hl, .mnemonic = "BIT 0,(HL)" };
        table[0x47] = .{ .handler = cb_bit_0_a, .mnemonic = "BIT 0,A" };

        // CB 0x48-0x4F: BIT 1, r
        table[0x48] = .{ .handler = cb_bit_1_b, .mnemonic = "BIT 1,B" };
        table[0x49] = .{ .handler = cb_bit_1_c, .mnemonic = "BIT 1,C" };
        table[0x4A] = .{ .handler = cb_bit_1_d, .mnemonic = "BIT 1,D" };
        table[0x4B] = .{ .handler = cb_bit_1_e, .mnemonic = "BIT 1,E" };
        table[0x4C] = .{ .handler = cb_bit_1_h, .mnemonic = "BIT 1,H" };
        table[0x4D] = .{ .handler = cb_bit_1_l, .mnemonic = "BIT 1,L" };
        table[0x4E] = .{ .handler = cb_bit_1_hl, .mnemonic = "BIT 1,(HL)" };
        table[0x4F] = .{ .handler = cb_bit_1_a, .mnemonic = "BIT 1,A" };

        // CB 0x50-0x57: BIT 2, r
        table[0x50] = .{ .handler = cb_bit_2_b, .mnemonic = "BIT 2,B" };
        table[0x51] = .{ .handler = cb_bit_2_c, .mnemonic = "BIT 2,C" };
        table[0x52] = .{ .handler = cb_bit_2_d, .mnemonic = "BIT 2,D" };
        table[0x53] = .{ .handler = cb_bit_2_e, .mnemonic = "BIT 2,E" };
        table[0x54] = .{ .handler = cb_bit_2_h, .mnemonic = "BIT 2,H" };
        table[0x55] = .{ .handler = cb_bit_2_l, .mnemonic = "BIT 2,L" };
        table[0x56] = .{ .handler = cb_bit_2_hl, .mnemonic = "BIT 2,(HL)" };
        table[0x57] = .{ .handler = cb_bit_2_a, .mnemonic = "BIT 2,A" };

        // CB 0x58-0x5F: BIT 3, r
        table[0x58] = .{ .handler = cb_bit_3_b, .mnemonic = "BIT 3,B" };
        table[0x59] = .{ .handler = cb_bit_3_c, .mnemonic = "BIT 3,C" };
        table[0x5A] = .{ .handler = cb_bit_3_d, .mnemonic = "BIT 3,D" };
        table[0x5B] = .{ .handler = cb_bit_3_e, .mnemonic = "BIT 3,E" };
        table[0x5C] = .{ .handler = cb_bit_3_h, .mnemonic = "BIT 3,H" };
        table[0x5D] = .{ .handler = cb_bit_3_l, .mnemonic = "BIT 3,L" };
        table[0x5E] = .{ .handler = cb_bit_3_hl, .mnemonic = "BIT 3,(HL)" };
        table[0x5F] = .{ .handler = cb_bit_3_a, .mnemonic = "BIT 3,A" };

        // CB 0x60-0x67: BIT 4, r
        table[0x60] = .{ .handler = cb_bit_4_b, .mnemonic = "BIT 4,B" };
        table[0x61] = .{ .handler = cb_bit_4_c, .mnemonic = "BIT 4,C" };
        table[0x62] = .{ .handler = cb_bit_4_d, .mnemonic = "BIT 4,D" };
        table[0x63] = .{ .handler = cb_bit_4_e, .mnemonic = "BIT 4,E" };
        table[0x64] = .{ .handler = cb_bit_4_h, .mnemonic = "BIT 4,H" };
        table[0x65] = .{ .handler = cb_bit_4_l, .mnemonic = "BIT 4,L" };
        table[0x66] = .{ .handler = cb_bit_4_hl, .mnemonic = "BIT 4,(HL)" };
        table[0x67] = .{ .handler = cb_bit_4_a, .mnemonic = "BIT 4,A" };

        // CB 0x68-0x6F: BIT 5, r
        table[0x68] = .{ .handler = cb_bit_5_b, .mnemonic = "BIT 5,B" };
        table[0x69] = .{ .handler = cb_bit_5_c, .mnemonic = "BIT 5,C" };
        table[0x6A] = .{ .handler = cb_bit_5_d, .mnemonic = "BIT 5,D" };
        table[0x6B] = .{ .handler = cb_bit_5_e, .mnemonic = "BIT 5,E" };
        table[0x6C] = .{ .handler = cb_bit_5_h, .mnemonic = "BIT 5,H" };
        table[0x6D] = .{ .handler = cb_bit_5_l, .mnemonic = "BIT 5,L" };
        table[0x6E] = .{ .handler = cb_bit_5_hl, .mnemonic = "BIT 5,(HL)" };
        table[0x6F] = .{ .handler = cb_bit_5_a, .mnemonic = "BIT 5,A" };

        // CB 0x70-0x77: BIT 6, r
        table[0x70] = .{ .handler = cb_bit_6_b, .mnemonic = "BIT 6,B" };
        table[0x71] = .{ .handler = cb_bit_6_c, .mnemonic = "BIT 6,C" };
        table[0x72] = .{ .handler = cb_bit_6_d, .mnemonic = "BIT 6,D" };
        table[0x73] = .{ .handler = cb_bit_6_e, .mnemonic = "BIT 6,E" };
        table[0x74] = .{ .handler = cb_bit_6_h, .mnemonic = "BIT 6,H" };
        table[0x75] = .{ .handler = cb_bit_6_l, .mnemonic = "BIT 6,L" };
        table[0x76] = .{ .handler = cb_bit_6_hl, .mnemonic = "BIT 6,(HL)" };
        table[0x77] = .{ .handler = cb_bit_6_a, .mnemonic = "BIT 6,A" };

        // CB 0x78-0x7F: BIT 7, r
        table[0x78] = .{ .handler = cb_bit_7_b, .mnemonic = "BIT 7,B" };
        table[0x79] = .{ .handler = cb_bit_7_c, .mnemonic = "BIT 7,C" };
        table[0x7A] = .{ .handler = cb_bit_7_d, .mnemonic = "BIT 7,D" };
        table[0x7B] = .{ .handler = cb_bit_7_e, .mnemonic = "BIT 7,E" };
        table[0x7C] = .{ .handler = cb_bit_7_h, .mnemonic = "BIT 7,H" };
        table[0x7D] = .{ .handler = cb_bit_7_l, .mnemonic = "BIT 7,L" };
        table[0x7E] = .{ .handler = cb_bit_7_hl, .mnemonic = "BIT 7,(HL)" };
        table[0x7F] = .{ .handler = cb_bit_7_a, .mnemonic = "BIT 7,A" };

        // CB 0x80-0x87: RES 0, r
        table[0x80] = .{ .handler = cb_res_0_b, .mnemonic = "RES 0,B" };
        table[0x81] = .{ .handler = cb_res_0_c, .mnemonic = "RES 0,C" };
        table[0x82] = .{ .handler = cb_res_0_d, .mnemonic = "RES 0,D" };
        table[0x83] = .{ .handler = cb_res_0_e, .mnemonic = "RES 0,E" };
        table[0x84] = .{ .handler = cb_res_0_h, .mnemonic = "RES 0,H" };
        table[0x85] = .{ .handler = cb_res_0_l, .mnemonic = "RES 0,L" };
        table[0x86] = .{ .handler = cb_res_0_hl, .mnemonic = "RES 0,(HL)" };
        table[0x87] = .{ .handler = cb_res_0_a, .mnemonic = "RES 0,A" };

        // CB 0x88-0x8F: RES 1, r
        table[0x88] = .{ .handler = cb_res_1_b, .mnemonic = "RES 1,B" };
        table[0x89] = .{ .handler = cb_res_1_c, .mnemonic = "RES 1,C" };
        table[0x8A] = .{ .handler = cb_res_1_d, .mnemonic = "RES 1,D" };
        table[0x8B] = .{ .handler = cb_res_1_e, .mnemonic = "RES 1,E" };
        table[0x8C] = .{ .handler = cb_res_1_h, .mnemonic = "RES 1,H" };
        table[0x8D] = .{ .handler = cb_res_1_l, .mnemonic = "RES 1,L" };
        table[0x8E] = .{ .handler = cb_res_1_hl, .mnemonic = "RES 1,(HL)" };
        table[0x8F] = .{ .handler = cb_res_1_a, .mnemonic = "RES 1,A" };

        // CB 0x90-0x97: RES 2, r
        table[0x90] = .{ .handler = cb_res_2_b, .mnemonic = "RES 2,B" };
        table[0x91] = .{ .handler = cb_res_2_c, .mnemonic = "RES 2,C" };
        table[0x92] = .{ .handler = cb_res_2_d, .mnemonic = "RES 2,D" };
        table[0x93] = .{ .handler = cb_res_2_e, .mnemonic = "RES 2,E" };
        table[0x94] = .{ .handler = cb_res_2_h, .mnemonic = "RES 2,H" };
        table[0x95] = .{ .handler = cb_res_2_l, .mnemonic = "RES 2,L" };
        table[0x96] = .{ .handler = cb_res_2_hl, .mnemonic = "RES 2,(HL)" };
        table[0x97] = .{ .handler = cb_res_2_a, .mnemonic = "RES 2,A" };

        // CB 0x98-0x9F: RES 3, r
        table[0x98] = .{ .handler = cb_res_3_b, .mnemonic = "RES 3,B" };
        table[0x99] = .{ .handler = cb_res_3_c, .mnemonic = "RES 3,C" };
        table[0x9A] = .{ .handler = cb_res_3_d, .mnemonic = "RES 3,D" };
        table[0x9B] = .{ .handler = cb_res_3_e, .mnemonic = "RES 3,E" };
        table[0x9C] = .{ .handler = cb_res_3_h, .mnemonic = "RES 3,H" };
        table[0x9D] = .{ .handler = cb_res_3_l, .mnemonic = "RES 3,L" };
        table[0x9E] = .{ .handler = cb_res_3_hl, .mnemonic = "RES 3,(HL)" };
        table[0x9F] = .{ .handler = cb_res_3_a, .mnemonic = "RES 3,A" };

        // CB 0xA0-0xA7: RES 4, r
        table[0xA0] = .{ .handler = cb_res_4_b, .mnemonic = "RES 4,B" };
        table[0xA1] = .{ .handler = cb_res_4_c, .mnemonic = "RES 4,C" };
        table[0xA2] = .{ .handler = cb_res_4_d, .mnemonic = "RES 4,D" };
        table[0xA3] = .{ .handler = cb_res_4_e, .mnemonic = "RES 4,E" };
        table[0xA4] = .{ .handler = cb_res_4_h, .mnemonic = "RES 4,H" };
        table[0xA5] = .{ .handler = cb_res_4_l, .mnemonic = "RES 4,L" };
        table[0xA6] = .{ .handler = cb_res_4_hl, .mnemonic = "RES 4,(HL)" };
        table[0xA7] = .{ .handler = cb_res_4_a, .mnemonic = "RES 4,A" };

        // CB 0xA8-0xAF: RES 5, r
        table[0xA8] = .{ .handler = cb_res_5_b, .mnemonic = "RES 5,B" };
        table[0xA9] = .{ .handler = cb_res_5_c, .mnemonic = "RES 5,C" };
        table[0xAA] = .{ .handler = cb_res_5_d, .mnemonic = "RES 5,D" };
        table[0xAB] = .{ .handler = cb_res_5_e, .mnemonic = "RES 5,E" };
        table[0xAC] = .{ .handler = cb_res_5_h, .mnemonic = "RES 5,H" };
        table[0xAD] = .{ .handler = cb_res_5_l, .mnemonic = "RES 5,L" };
        table[0xAE] = .{ .handler = cb_res_5_hl, .mnemonic = "RES 5,(HL)" };
        table[0xAF] = .{ .handler = cb_res_5_a, .mnemonic = "RES 5,A" };

        // CB 0xB0-0xB7: RES 6, r
        table[0xB0] = .{ .handler = cb_res_6_b, .mnemonic = "RES 6,B" };
        table[0xB1] = .{ .handler = cb_res_6_c, .mnemonic = "RES 6,C" };
        table[0xB2] = .{ .handler = cb_res_6_d, .mnemonic = "RES 6,D" };
        table[0xB3] = .{ .handler = cb_res_6_e, .mnemonic = "RES 6,E" };
        table[0xB4] = .{ .handler = cb_res_6_h, .mnemonic = "RES 6,H" };
        table[0xB5] = .{ .handler = cb_res_6_l, .mnemonic = "RES 6,L" };
        table[0xB6] = .{ .handler = cb_res_6_hl, .mnemonic = "RES 6,(HL)" };
        table[0xB7] = .{ .handler = cb_res_6_a, .mnemonic = "RES 6,A" };

        // CB 0xB8-0xBF: RES 7, r
        table[0xB8] = .{ .handler = cb_res_7_b, .mnemonic = "RES 7,B" };
        table[0xB9] = .{ .handler = cb_res_7_c, .mnemonic = "RES 7,C" };
        table[0xBA] = .{ .handler = cb_res_7_d, .mnemonic = "RES 7,D" };
        table[0xBB] = .{ .handler = cb_res_7_e, .mnemonic = "RES 7,E" };
        table[0xBC] = .{ .handler = cb_res_7_h, .mnemonic = "RES 7,H" };
        table[0xBD] = .{ .handler = cb_res_7_l, .mnemonic = "RES 7,L" };
        table[0xBE] = .{ .handler = cb_res_7_hl, .mnemonic = "RES 7,(HL)" };
        table[0xBF] = .{ .handler = cb_res_7_a, .mnemonic = "RES 7,A" };

        // CB 0xC0-0xC7: SET 0, r
        table[0xC0] = .{ .handler = cb_set_0_b, .mnemonic = "SET 0,B" };
        table[0xC1] = .{ .handler = cb_set_0_c, .mnemonic = "SET 0,C" };
        table[0xC2] = .{ .handler = cb_set_0_d, .mnemonic = "SET 0,D" };
        table[0xC3] = .{ .handler = cb_set_0_e, .mnemonic = "SET 0,E" };
        table[0xC4] = .{ .handler = cb_set_0_h, .mnemonic = "SET 0,H" };
        table[0xC5] = .{ .handler = cb_set_0_l, .mnemonic = "SET 0,L" };
        table[0xC6] = .{ .handler = cb_set_0_hl, .mnemonic = "SET 0,(HL)" };
        table[0xC7] = .{ .handler = cb_set_0_a, .mnemonic = "SET 0,A" };

        // CB 0xC8-0xCF: SET 1, r
        table[0xC8] = .{ .handler = cb_set_1_b, .mnemonic = "SET 1,B" };
        table[0xC9] = .{ .handler = cb_set_1_c, .mnemonic = "SET 1,C" };
        table[0xCA] = .{ .handler = cb_set_1_d, .mnemonic = "SET 1,D" };
        table[0xCB] = .{ .handler = cb_set_1_e, .mnemonic = "SET 1,E" };
        table[0xCC] = .{ .handler = cb_set_1_h, .mnemonic = "SET 1,H" };
        table[0xCD] = .{ .handler = cb_set_1_l, .mnemonic = "SET 1,L" };
        table[0xCE] = .{ .handler = cb_set_1_hl, .mnemonic = "SET 1,(HL)" };
        table[0xCF] = .{ .handler = cb_set_1_a, .mnemonic = "SET 1,A" };

        // CB 0xD0-0xD7: SET 2, r
        table[0xD0] = .{ .handler = cb_set_2_b, .mnemonic = "SET 2,B" };
        table[0xD1] = .{ .handler = cb_set_2_c, .mnemonic = "SET 2,C" };
        table[0xD2] = .{ .handler = cb_set_2_d, .mnemonic = "SET 2,D" };
        table[0xD3] = .{ .handler = cb_set_2_e, .mnemonic = "SET 2,E" };
        table[0xD4] = .{ .handler = cb_set_2_h, .mnemonic = "SET 2,H" };
        table[0xD5] = .{ .handler = cb_set_2_l, .mnemonic = "SET 2,L" };
        table[0xD6] = .{ .handler = cb_set_2_hl, .mnemonic = "SET 2,(HL)" };
        table[0xD7] = .{ .handler = cb_set_2_a, .mnemonic = "SET 2,A" };

        // CB 0xD8-0xDF: SET 3, r
        table[0xD8] = .{ .handler = cb_set_3_b, .mnemonic = "SET 3,B" };
        table[0xD9] = .{ .handler = cb_set_3_c, .mnemonic = "SET 3,C" };
        table[0xDA] = .{ .handler = cb_set_3_d, .mnemonic = "SET 3,D" };
        table[0xDB] = .{ .handler = cb_set_3_e, .mnemonic = "SET 3,E" };
        table[0xDC] = .{ .handler = cb_set_3_h, .mnemonic = "SET 3,H" };
        table[0xDD] = .{ .handler = cb_set_3_l, .mnemonic = "SET 3,L" };
        table[0xDE] = .{ .handler = cb_set_3_hl, .mnemonic = "SET 3,(HL)" };
        table[0xDF] = .{ .handler = cb_set_3_a, .mnemonic = "SET 3,A" };

        // CB 0xE0-0xE7: SET 4, r
        table[0xE0] = .{ .handler = cb_set_4_b, .mnemonic = "SET 4,B" };
        table[0xE1] = .{ .handler = cb_set_4_c, .mnemonic = "SET 4,C" };
        table[0xE2] = .{ .handler = cb_set_4_d, .mnemonic = "SET 4,D" };
        table[0xE3] = .{ .handler = cb_set_4_e, .mnemonic = "SET 4,E" };
        table[0xE4] = .{ .handler = cb_set_4_h, .mnemonic = "SET 4,H" };
        table[0xE5] = .{ .handler = cb_set_4_l, .mnemonic = "SET 4,L" };
        table[0xE6] = .{ .handler = cb_set_4_hl, .mnemonic = "SET 4,(HL)" };
        table[0xE7] = .{ .handler = cb_set_4_a, .mnemonic = "SET 4,A" };

        // CB 0xE8-0xEF: SET 5, r
        table[0xE8] = .{ .handler = cb_set_5_b, .mnemonic = "SET 5,B" };
        table[0xE9] = .{ .handler = cb_set_5_c, .mnemonic = "SET 5,C" };
        table[0xEA] = .{ .handler = cb_set_5_d, .mnemonic = "SET 5,D" };
        table[0xEB] = .{ .handler = cb_set_5_e, .mnemonic = "SET 5,E" };
        table[0xEC] = .{ .handler = cb_set_5_h, .mnemonic = "SET 5,H" };
        table[0xED] = .{ .handler = cb_set_5_l, .mnemonic = "SET 5,L" };
        table[0xEE] = .{ .handler = cb_set_5_hl, .mnemonic = "SET 5,(HL)" };
        table[0xEF] = .{ .handler = cb_set_5_a, .mnemonic = "SET 5,A" };

        // CB 0xF0-0xF7: SET 6, r
        table[0xF0] = .{ .handler = cb_set_6_b, .mnemonic = "SET 6,B" };
        table[0xF1] = .{ .handler = cb_set_6_c, .mnemonic = "SET 6,C" };
        table[0xF2] = .{ .handler = cb_set_6_d, .mnemonic = "SET 6,D" };
        table[0xF3] = .{ .handler = cb_set_6_e, .mnemonic = "SET 6,E" };
        table[0xF4] = .{ .handler = cb_set_6_h, .mnemonic = "SET 6,H" };
        table[0xF5] = .{ .handler = cb_set_6_l, .mnemonic = "SET 6,L" };
        table[0xF6] = .{ .handler = cb_set_6_hl, .mnemonic = "SET 6,(HL)" };
        table[0xF7] = .{ .handler = cb_set_6_a, .mnemonic = "SET 6,A" };

        // CB 0xF8-0xFF: SET 7, r
        table[0xF8] = .{ .handler = cb_set_7_b, .mnemonic = "SET 7,B" };
        table[0xF9] = .{ .handler = cb_set_7_c, .mnemonic = "SET 7,C" };
        table[0xFA] = .{ .handler = cb_set_7_d, .mnemonic = "SET 7,D" };
        table[0xFB] = .{ .handler = cb_set_7_e, .mnemonic = "SET 7,E" };
        table[0xFC] = .{ .handler = cb_set_7_h, .mnemonic = "SET 7,H" };
        table[0xFD] = .{ .handler = cb_set_7_l, .mnemonic = "SET 7,L" };
        table[0xFE] = .{ .handler = cb_set_7_hl, .mnemonic = "SET 7,(HL)" };
        table[0xFF] = .{ .handler = cb_set_7_a, .mnemonic = "SET 7,A" };

        break :init table;
    };

    /// Execute CB-prefixed instruction using lookup table
    fn executeCBInstruction(self: *CPU, system_bus: *SystemBus) !u32 {
        const cb_opcode = try system_bus.read(self.PC);
        self.PC +%= 1;

        // Increment CB instruction counter for this CPU instance
        self.cb_opcode_counts[cb_opcode] +%= 1;
        const instruction = CB_OPCODE_TABLE[cb_opcode];
        return try instruction.handler(self, system_bus);
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

        const opcode = try system_bus.read(self.PC);
        self.PC +%= 1;

        const is_cb_instr = opcode == 0xCB;

        // Increment instruction execution count for this CPU instance
        self.opcode_counts[opcode] +%= 1;
        if (is_cb_instr) {
            return try executeCBInstruction(self, system_bus);
        }
        const instruction = OPCODE_TABLE[opcode];
        return try instruction.handler(self, system_bus);
    }

    /// Get the execution count for a specific opcode
    pub fn getOpcodeCount(self: *const CPU, opcode: u8) u64 {
        return self.opcode_counts[opcode];
    }

    /// Get the mnemonic for a specific opcode
    pub fn getOpcodeMnemonic(opcode: u8) []const u8 {
        return OPCODE_TABLE[opcode].mnemonic;
    }

    /// Get the execution count for a specific CB opcode
    pub fn getCBOpcodeCount(self: *const CPU, cb_opcode: u8) u64 {
        return self.cb_opcode_counts[cb_opcode];
    }

    /// Get the mnemonic for a specific CB opcode
    pub fn getCBOpcodeMnemonic(cb_opcode: u8) []const u8 {
        return CB_OPCODE_TABLE[cb_opcode].mnemonic;
    }

    /// Print instruction statistics to stderr
    pub fn printInstructionStats(self: *const CPU) void {
        std.debug.print("\n=== Instruction Execution Statistics ===\n", .{});
        var total: u64 = 0;

        std.debug.print("\n--- Unprefixed Instructions ---\n", .{});
        for (OPCODE_TABLE, 0..) |instr, opcode| {
            const count = self.opcode_counts[opcode];
            if (count > 0) {
                std.debug.print("0x{X:0>2} {s:<20} : {d} times\n", .{
                    opcode,
                    instr.mnemonic,
                    count,
                });
                total += count;
            }
        }

        std.debug.print("\n--- CB-Prefixed Instructions ---\n", .{});
        for (CB_OPCODE_TABLE, 0..) |instr, opcode| {
            const count = self.cb_opcode_counts[opcode];
            if (count > 0) {
                std.debug.print("CB {X:0>2} {s:<20} : {d} times\n", .{
                    opcode,
                    instr.mnemonic,
                    count,
                });
                total += count;
            }
        }

        std.debug.print("\nTotal instructions executed: {d}\n", .{total});
        std.debug.print("========================================\n\n", .{});
    }

    pub fn printCpuState(self: CPU) void {
        std.debug.print("{}", .{self});
    }

    /// Reset all instruction counters to zero for this CPU instance
    pub fn resetInstructionCounts(self: *CPU) void {
        for (&self.opcode_counts) |*count| {
            count.* = 0;
        }
        for (&self.cb_opcode_counts) |*count| {
            count.* = 0;
        }
    }
};

test "run whole boot ROM" {
    // 1. Set up system bus
    var sysbus = &@import("system_bus.zig").g_test_system_bus;
    try sysbus.init(sysbus.mappings);

    // 2. Wire MMIO bus slice
    cart_mmio.setBus(&sysbus.bus.mem);

    // 3. Load boot ROM into BootRom
    const rom_buf = try loader.loadFixed256Rom("../roms/dmg_boot.bin");
    var boot = BootRom.init(rom_buf[0..]);

    // 4. Build a tiny fake cartridge just for logo + header
    var cart_bytes: [0x150]u8 = [_]u8{0} ** 0x150;

    const nintendo_logo = [_]u8{
        0xCE, 0xED, 0x66, 0x66, 0xCC, 0x0D, 0x00, 0x0B,
        0x03, 0x73, 0x00, 0x83, 0x00, 0x0C, 0x00, 0x0D,
        0x00, 0x08, 0x11, 0x1F, 0x88, 0x89, 0x00, 0x0E,
        0xDC, 0xCC, 0x6E, 0xE6, 0xDD, 0xDD, 0xD9, 0x99,
        0xBB, 0xBB, 0x67, 0x63, 0x6E, 0x0E, 0xEC, 0xCC,
        0xDD, 0xDC, 0x99, 0x9F, 0xBB, 0xB9, 0x33, 0x3E,
    };
    for (nintendo_logo, 0..) |b, i| {
        cart_bytes[0x0104 + i] = b;
    }

    const header_bytes = [_]u8{
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xE7,
    };
    for (header_bytes, 0..) |b, i| {
        cart_bytes[0x0134 + i] = b;
    }

    var cart = Cartridge.init(cart_bytes[0..]);

    // 5. Plug BootRom + Cartridge into MMIO
    cart_mmio.setCartridgeBoot(&cart, &boot);
    // leave boot ROM enabled so 0x0000–0x00FF are served by BootRom.read

    // 6. CPU
    var cpu = CPU.init();
    var cycles: usize = 0;

    while (cpu.PC != 0x0100) {
        const c = try cpu.step(sysbus);
        cycles += c;
    }
    try std.testing.expectEqual(@as(u16, 0x0100), cpu.PC);
    try std.testing.expectEqual(@as(u16, 0xFFFE), cpu.SP);

    try std.testing.expectEqual(@as(u16, 0x01B0), CPU.get16BitRegister(cpu.A, cpu.F));
    try std.testing.expectEqual(@as(u16, 0x0013), CPU.get16BitRegister(cpu.B, cpu.C));
    try std.testing.expectEqual(@as(u16, 0x00D8), CPU.get16BitRegister(cpu.D, cpu.E));
    try std.testing.expectEqual(@as(u16, 0x014D), CPU.get16BitRegister(cpu.H, cpu.L));

    try std.testing.expectEqual(@as(u8, 0x01), cpu.A);
    try std.testing.expectEqual(@as(u8, 0xB0), cpu.F);

    try std.testing.expectEqual(@as(u8, 0x00), cpu.B);
    try std.testing.expectEqual(@as(u8, 0x13), cpu.C);

    try std.testing.expectEqual(@as(u8, 0x00), cpu.D);
    try std.testing.expectEqual(@as(u8, 0xD8), cpu.E);

    try std.testing.expectEqual(@as(u8, 0x01), cpu.H);
    try std.testing.expectEqual(@as(u8, 0x4D), cpu.L);

    // Flags in F = 0xB0 -> 1011_0000: Z=1, N=0, H=1, C=1
    try std.testing.expect(cpu.getFlag(CPU.Z_FLAG));
    try std.testing.expect(!cpu.getFlag(CPU.N_FLAG));
    try std.testing.expect(cpu.getFlag(CPU.H_FLAG));
    try std.testing.expect(cpu.getFlag(CPU.C_FLAG));

    // Optionally, sanity check halted state
    try std.testing.expect(!cpu.halted);

    cpu.printInstructionStats();
}
