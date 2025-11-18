const std = @import("std");
const windowing = @import("window.zig");
const loader = @import("loader.zig");
const expect = std.testing.expect;

comptime {
    _ = @import("system_bus.zig");
}

pub fn main() !void {
    // try windowing.makeWindow(640, 380);

    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);
    const rom_buf = try loader.loadRomFromRomsDir();
    for (rom_buf, 0..0x100) |b, i| {
        try bus.write(@intCast(i), b);
    }

    std.debug.print("=== Game Boy Emulator - Boot ROM Verification ===\n\n", .{});

    var cpu = @import("cpu.zig").CPU.init();

    // comptime var cycles = 0;
    for (0..0x100) |_| {
        _ = try cpu.step(&bus);
    }

    std.debug.print("{}", .{cpu});

    return;
}

test "Basic test" {
    try expect(1 == 1);
}
