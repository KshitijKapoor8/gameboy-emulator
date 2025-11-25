const std = @import("std");
const expect = std.testing.expect;

const CPU = @import("cpu.zig").CPU;
const Bus = @import("bus.zig").Bus;
const SystemBus = @import("system_bus.zig").SystemBus;
const MEMORY_MAP = @import("system_bus.zig").MEMORY_MAP;

const Cartridge = @import("devices//cartridge.zig").Cartridge;
const BootRom = @import("devices/bootrom.zig").BootRom;
const cart_mmio = @import("devices/cart_boot_mmio.zig");
const loader = @import("loader.zig");
const mmio = @import("devices/mmio.zig"); // for serialReset/serialSlice

fn serialContainsPassed() bool {
    const buf = mmio.serialSlice();
    return std.mem.indexOf(u8, buf, "Passed") != null;
}

test "blargg 01-special runs and prints Passed" {
    const allocator = std.testing.allocator;

    const rom_data = try loader.loadCartRomFromPath(
        allocator,
        "../roms/01-special.gb",
    );
    defer allocator.free(rom_data);

    var cart = Cartridge.init(rom_data);

    var dummy_boot: BootRom = undefined;

    var local_bus = Bus.init();
    cart_mmio.setBus(local_bus.mem[0..]);
    cart_mmio.setCartridgeBoot(&cart, &dummy_boot);
    cart_mmio.disableBootRom(); // IMPORTANT for blargg

    var sysbus = SystemBus{
        .bus = &local_bus,
        .mappings = MEMORY_MAP[0..],
    };
    try sysbus.init(MEMORY_MAP[0..]);

    var cpu = CPU.initPostBoot();

    // Reset serial log
    mmio.serialReset();

    const max_steps: usize = 10_000_000;
    var steps: usize = 0;

    while (steps < max_steps and !serialContainsPassed()) : (steps += 1) {
        const used_cycles = try cpu.step(&sysbus);
        _ = used_cycles; // ignoring timing here
    }

    const buf = mmio.serialSlice();
    std.debug.print("\n[serial output]\n{s}\n[/serial output]\n", .{buf});

    try expect(serialContainsPassed());
}
