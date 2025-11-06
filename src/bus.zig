const std = @import("std");
const expect = std.testing.expect;
const expectError = std.testing.expectError;

pub fn Bus64KB() type {
    return BusFactory(0x10000);
}

pub fn Bus(comptime size: usize) type {
    return BusFactory(size);
}

// creates a Bus of specified size
fn BusFactory(comptime size_req: ?usize) type {
    const size: usize = size_req orelse 0x10000;

    return struct {
        const Self = @This();
        mem: [size]u8 = .{0} ** size,
        size: usize = size,

        pub fn init() Self {
            return .{};
        }

        pub fn read(self: *Self, addr: u16) u8 {
            return @intCast(self.mem[addr]);
        }
        pub fn write(self: *Self, addr: u16, val: u8) void {
            self.mem[addr] = val;
        }

        pub fn writeBuffer(self: *Self, addr: u16, buffer: []const u8) !void {
            if (buffer.len + addr > self.size) {
                return error.OutOfMemory;
            }
            for (0..buffer.len, buffer) |i, byte| {
                const idx: u16 = @intCast(i);
                self.write(addr + idx, byte);
            }
        }
    };
}

test "Bus Read and Write" {
    var bus = Bus64KB().init();
    const TEST_VALUE: u8 = 0xA;
    bus.write(0, TEST_VALUE);
    try expect(bus.read(0) == TEST_VALUE);
}

test "Bus Write Buffer" {
    var bus = Bus64KB().init();
    const TEST_LENGTH: u9 = 0x100;
    const TEST_VALUE: u8 = 0xA;
    const TEST_VALUES: [TEST_LENGTH]u8 = .{TEST_VALUE} ** TEST_LENGTH;

    const addr = 0;

    try bus.writeBuffer(addr, &TEST_VALUES);
    for (0..TEST_LENGTH) |i| {
        const idx: u16 = @intCast(i);
        try expect(bus.read(addr + idx) == TEST_VALUE);
    }
}

test "Bus OOM" {
    var bus = Bus(0x10).init();
    const TEST_LENGTH: u9 = 0x10;
    const TEST_VALUE: u8 = 0xA;
    const TEST_VALUES: [TEST_LENGTH]u8 = .{TEST_VALUE} ** TEST_LENGTH;

    const addr_ok: comptime_int = 0;

    try bus.writeBuffer(addr_ok, &TEST_VALUES);
    for (0..TEST_LENGTH) |i| {
        const idx: u16 = @intCast(i);
        try expect(bus.read(addr_ok + idx) == TEST_VALUE);
    }

    const addr_err: comptime_int = 1;

    try expectError(error.OutOfMemory, bus.writeBuffer(addr_err, &TEST_VALUES));
}
