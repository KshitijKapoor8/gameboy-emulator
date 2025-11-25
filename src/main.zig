const std = @import("std");
const windowing = @import("window.zig");
const loader = @import("loader.zig");
const expect = std.testing.expect;
const BootRom = @import("devices/bootrom.zig").BootRom;
const Cartridge = @import("devices/cartridge.zig").Cartridge;
const cart_mmio = @import("devices/cart_boot_mmio.zig");

pub fn main() !void {
    // try windowing.makeWindow(640, 380);

    var bus = @import("system_bus.zig").g_test_system_bus;
    try bus.init(bus.mappings);
    const rom_buf = try loader.loadRomFromRomsDir();
    cart_mmio.setBus(&bus.bus.mem);
    var boot = BootRom.init(rom_buf[0..]);
    for (rom_buf, 0..0x100) |b, i| {
        try bus.write(@intCast(i), b);
    }
    const gpa = std.heap.page_allocator;
    const cart_bytes = try loader.loadCartRomAlloc(gpa);
    var cart = Cartridge.init(cart_bytes);

    cart_mmio.setCartridgeBoot(&cart, &boot);

    std.debug.print("=== Game Boy Emulator - Boot ROM Verification ===\n\n", .{});

    var cpu = @import("cpu.zig").CPU.init();

    // comptime var cycles = 0;
    for (0..0x100) |_| {
        _ = try cpu.step(&bus);
    }

    // std.debug.print("{}", .{cpu});

    return;
}

test "Basic test" {
    try expect(1 == 1);
}
