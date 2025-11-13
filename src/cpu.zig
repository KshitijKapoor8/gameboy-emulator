const std = @import("std");
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

    /// Read 16-bit immediate value from PC
    inline fn read16Immediate(self: *CPU, system_bus: *SystemBus) !u16 {
        const low = try system_bus.read(self.PC);
        self.PC +%= 1;
        const high = try system_bus.read(self.PC);
        self.PC +%= 1;
        return get16BitRegister(high, low);
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
        self.SP -%= 1;
        try system_bus.write(self.SP, self.B);
        self.SP -%= 1;
        try system_bus.write(self.SP, self.C);
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
        const sp = self.SP;
        const result = if (offset >= 0) sp +% @as(u16, @intCast(offset)) else sp -% @as(u16, @intCast(-offset));

        self.setFlag(Z_FLAG, false);
        self.setFlag(N_FLAG, false);
        // For ADD SP,e8, H and C flags are calculated on lower byte
        if (offset >= 0) {
            self.setFlag(H_FLAG, ((sp & 0x0F) + (@as(u16, @intCast(offset)) & 0x0F)) > 0x0F);
            self.setFlag(C_FLAG, ((sp & 0xFF) + (@as(u16, @intCast(offset)) & 0xFF)) > 0xFF);
        } else {
            self.setFlag(H_FLAG, (result & 0x0F) > (sp & 0x0F));
            self.setFlag(C_FLAG, (result & 0xFF) > (sp & 0xFF));
        }
        self.SP = result;
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
        const sp = self.SP;
        const result = if (offset >= 0) sp +% @as(u16, @intCast(offset)) else sp -% @as(u16, @intCast(-offset));

        self.setFlag(Z_FLAG, false);
        self.setFlag(N_FLAG, false);
        if (offset >= 0) {
            self.setFlag(H_FLAG, ((sp & 0x0F) + (@as(u16, @intCast(offset)) & 0x0F)) > 0x0F);
            self.setFlag(C_FLAG, ((sp & 0xFF) + (@as(u16, @intCast(offset)) & 0xFF)) > 0xFF);
        } else {
            self.setFlag(H_FLAG, (result & 0x0F) > (sp & 0x0F));
            self.setFlag(C_FLAG, (result & 0xFF) > (sp & 0xFF));
        }
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
        _ = self;
        _ = system_bus;
        return error.UnimplementedOpcode;
    }

    // ==================== Opcode Lookup Table ====================

    const OPCODE_TABLE = init: {
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

        table[0xC0] = .{ .handler = op_ret, .mnemonic = "RET NZ" }; // Simplified for now
        table[0xC1] = .{ .handler = op_pop_bc, .mnemonic = "POP BC" };
        table[0xC2] = .{ .handler = op_jp_nz_nn, .mnemonic = "JP NZ,nn" };
        table[0xC3] = .{ .handler = op_jp_nn, .mnemonic = "JP nn" };
        table[0xC5] = .{ .handler = op_push_bc, .mnemonic = "PUSH BC" };
        table[0xC6] = .{ .handler = op_add_a_n8, .mnemonic = "ADD A,n8" };
        table[0xC9] = .{ .handler = op_ret, .mnemonic = "RET" };
        table[0xCA] = .{ .handler = op_jp_z_nn, .mnemonic = "JP Z,nn" };
        table[0xCD] = .{ .handler = op_call_nn, .mnemonic = "CALL nn" };

        table[0xFE] = .{ .handler = op_cp_a_n8, .mnemonic = "CP A,n8" };

        break :init table;
    };

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

        const instruction = OPCODE_TABLE[opcode];
        return try instruction.handler(self, system_bus);
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

test "INC and DEC instructions" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);

    var cpu = CPU{
        .A = 0x00,
        .F = 0,
        .B = 0x0F,
        .C = 0x01,
        .D = 0,
        .E = 0,
        .H = 0,
        .L = 0,
        .SP = 0xFFFE,
        .PC = 0x0100,
        .halted = false,
    };

    // INC B (0x0F -> 0x10, should set H flag)
    try bus.write(0x0100, 0x04);
    _ = try cpu.step(&bus);
    try expect(cpu.B == 0x10);
    try expect(cpu.getFlag(CPU.H_FLAG));
    try expect(!cpu.getFlag(CPU.Z_FLAG));

    // DEC C (0x01 -> 0x00, should set Z flag)
    try bus.write(0x0101, 0x0D);
    _ = try cpu.step(&bus);
    try expect(cpu.C == 0x00);
    try expect(cpu.getFlag(CPU.Z_FLAG));
    try expect(cpu.getFlag(CPU.N_FLAG));

    // INC A (0x00 -> 0x01)
    try bus.write(0x0102, 0x3C);
    _ = try cpu.step(&bus);
    try expect(cpu.A == 0x01);
    try expect(!cpu.getFlag(CPU.Z_FLAG));
}

test "16-bit load instructions" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);

    var cpu = CPU{
        .A = 0,
        .F = 0,
        .B = 0,
        .C = 0,
        .D = 0,
        .E = 0,
        .H = 0,
        .L = 0,
        .SP = 0xFFFE,
        .PC = 0x0100,
        .halted = false,
    };

    // LD HL, 0x1234
    try bus.write(0x0100, 0x21); // LD HL, n16
    try bus.write(0x0101, 0x34);
    try bus.write(0x0102, 0x12);
    var cycles = try cpu.step(&bus);
    try expect(cycles == 12);
    try expect(cpu.H == 0x12);
    try expect(cpu.L == 0x34);

    // LD SP, 0xFFF8
    try bus.write(0x0103, 0x31); // LD SP, n16
    try bus.write(0x0104, 0xF8);
    try bus.write(0x0105, 0xFF);
    cycles = try cpu.step(&bus);
    try expect(cycles == 12);
    try expect(cpu.SP == 0xFFF8);
}

test "LD (HL+) and LD (HL-) instructions" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);

    var cpu = CPU{
        .A = 0x42,
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

    // LD (HL+), A - Store A at HL then increment
    try bus.write(0x0100, 0x22);
    _ = try cpu.step(&bus);
    const stored = try bus.read(0xC000);
    try expect(stored == 0x42);
    try expect(CPU.get16BitRegister(cpu.H, cpu.L) == 0xC001);

    // LD A, (HL-) - Load from HL into A then decrement
    try bus.write(0xC001, 0x99);
    try bus.write(0x0101, 0x3A);
    _ = try cpu.step(&bus);
    try expect(cpu.A == 0x99);
    try expect(CPU.get16BitRegister(cpu.H, cpu.L) == 0xC000);
}

test "ADD and SUB instructions" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);

    var cpu = CPU{
        .A = 0x10,
        .F = 0,
        .B = 0x05,
        .C = 0x20,
        .D = 0,
        .E = 0,
        .H = 0,
        .L = 0,
        .SP = 0xFFFE,
        .PC = 0x0100,
        .halted = false,
    };

    // ADD A, B (0x10 + 0x05 = 0x15)
    try bus.write(0x0100, 0x80);
    _ = try cpu.step(&bus);
    try expect(cpu.A == 0x15);
    try expect(!cpu.getFlag(CPU.Z_FLAG));
    try expect(!cpu.getFlag(CPU.C_FLAG));

    // ADD A, C (0x15 + 0x20 = 0x35)
    try bus.write(0x0101, 0x81);
    _ = try cpu.step(&bus);
    try expect(cpu.A == 0x35);

    // SUB A, B (0x35 - 0x05 = 0x30)
    try bus.write(0x0102, 0x90);
    _ = try cpu.step(&bus);
    try expect(cpu.A == 0x30);
    try expect(cpu.getFlag(CPU.N_FLAG));

    // ADD A, n8 (0x30 + 0x50 = 0x80)
    try bus.write(0x0103, 0xC6);
    try bus.write(0x0104, 0x50);
    _ = try cpu.step(&bus);
    try expect(cpu.A == 0x80);
}

test "XOR A instruction" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);

    var cpu = CPU{
        .A = 0xFF,
        .F = 0xFF,
        .B = 0,
        .C = 0,
        .D = 0,
        .E = 0,
        .H = 0,
        .L = 0,
        .SP = 0xFFFE,
        .PC = 0x0100,
        .halted = false,
    };

    // XOR A - Should clear A and set Z flag
    try bus.write(0x0100, 0xAF);
    _ = try cpu.step(&bus);
    try expect(cpu.A == 0);
    try expect(cpu.getFlag(CPU.Z_FLAG));
    try expect(!cpu.getFlag(CPU.N_FLAG));
    try expect(!cpu.getFlag(CPU.H_FLAG));
    try expect(!cpu.getFlag(CPU.C_FLAG));
}

test "CP instruction" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);

    var cpu = CPU{
        .A = 0x10,
        .F = 0,
        .B = 0x10,
        .C = 0,
        .D = 0,
        .E = 0,
        .H = 0,
        .L = 0,
        .SP = 0xFFFE,
        .PC = 0x0100,
        .halted = false,
    };

    // CP A, B - A == B, should set Z flag
    try bus.write(0x0100, 0xB8);
    _ = try cpu.step(&bus);
    try expect(cpu.A == 0x10); // A should not change
    try expect(cpu.getFlag(CPU.Z_FLAG));
    try expect(cpu.getFlag(CPU.N_FLAG));

    // CP A, n8 - Compare with different value
    try bus.write(0x0101, 0xFE);
    try bus.write(0x0102, 0x05);
    _ = try cpu.step(&bus);
    try expect(!cpu.getFlag(CPU.Z_FLAG));
    try expect(!cpu.getFlag(CPU.C_FLAG));
}

test "JP and JR instructions" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);

    var cpu = CPU{
        .A = 0,
        .F = 0,
        .B = 0,
        .C = 0,
        .D = 0,
        .E = 0,
        .H = 0,
        .L = 0,
        .SP = 0xFFFE,
        .PC = 0x0100,
        .halted = false,
    };

    // JP 0x0200
    try bus.write(0x0100, 0xC3);
    try bus.write(0x0101, 0x00);
    try bus.write(0x0102, 0x02);
    var cycles = try cpu.step(&bus);
    try expect(cycles == 16);
    try expect(cpu.PC == 0x0200);

    // JR +5
    cpu.PC = 0x0200;
    try bus.write(0x0200, 0x18);
    try bus.write(0x0201, 0x05);
    cycles = try cpu.step(&bus);
    try expect(cycles == 12);
    try expect(cpu.PC == 0x0207);

    // JR NZ, +3 (Z flag not set, should jump)
    cpu.PC = 0x0300;
    cpu.setFlag(CPU.Z_FLAG, false);
    try bus.write(0x0300, 0x20);
    try bus.write(0x0301, 0x03);
    cycles = try cpu.step(&bus);
    try expect(cycles == 12);
    try expect(cpu.PC == 0x0305);

    // JR Z, +3 (Z flag not set, should NOT jump)
    cpu.PC = 0x0400;
    cpu.setFlag(CPU.Z_FLAG, false);
    try bus.write(0x0400, 0x28);
    try bus.write(0x0401, 0x03);
    cycles = try cpu.step(&bus);
    try expect(cycles == 8);
    try expect(cpu.PC == 0x0402);
}

test "PUSH and POP instructions" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);

    var cpu = CPU{
        .A = 0,
        .F = 0,
        .B = 0x12,
        .C = 0x34,
        .D = 0,
        .E = 0,
        .H = 0,
        .L = 0,
        .SP = 0xFFFE,
        .PC = 0x0100,
        .halted = false,
    };

    // PUSH BC
    try bus.write(0x0100, 0xC5);
    var cycles = try cpu.step(&bus);
    try expect(cycles == 16);
    try expect(cpu.SP == 0xFFFC);
    const low = try bus.read(0xFFFC);
    const high = try bus.read(0xFFFD);
    try expect(low == 0x34);
    try expect(high == 0x12);

    // Clear BC
    cpu.B = 0;
    cpu.C = 0;

    // POP BC
    try bus.write(0x0101, 0xC1);
    cycles = try cpu.step(&bus);
    try expect(cycles == 12);
    try expect(cpu.SP == 0xFFFE);
    try expect(cpu.B == 0x12);
    try expect(cpu.C == 0x34);
}

test "CALL and RET instructions" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);

    var cpu = CPU{
        .A = 0,
        .F = 0,
        .B = 0,
        .C = 0,
        .D = 0,
        .E = 0,
        .H = 0,
        .L = 0,
        .SP = 0xFFFE,
        .PC = 0x0100,
        .halted = false,
    };

    // CALL 0x0200
    try bus.write(0x0100, 0xCD);
    try bus.write(0x0101, 0x00);
    try bus.write(0x0102, 0x02);
    var cycles = try cpu.step(&bus);
    try expect(cycles == 24);
    try expect(cpu.PC == 0x0200);
    try expect(cpu.SP == 0xFFFC);

    // Check return address on stack (should be 0x0103)
    const ret_low = try bus.read(0xFFFC);
    const ret_high = try bus.read(0xFFFD);
    try expect(ret_low == 0x03);
    try expect(ret_high == 0x01);

    // RET
    try bus.write(0x0200, 0xC9);
    cycles = try cpu.step(&bus);
    try expect(cycles == 16);
    try expect(cpu.PC == 0x0103);
    try expect(cpu.SP == 0xFFFE);
}

test "HALT instruction" {
    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);

    var cpu = CPU{
        .A = 0,
        .F = 0,
        .B = 0,
        .C = 0,
        .D = 0,
        .E = 0,
        .H = 0,
        .L = 0,
        .SP = 0xFFFE,
        .PC = 0x0100,
        .halted = false,
    };

    // HALT
    try bus.write(0x0100, 0x76);
    _ = try cpu.step(&bus);
    try expect(cpu.halted);

    // Further steps should return 4 cycles without advancing
    const cycles = try cpu.step(&bus);
    try expect(cycles == 4);
    try expect(cpu.PC == 0x0101); // PC still at same position
}
