const std = @import("std");
const BootRom = @import("bootrom.zig").BootRom;
const Cartridge = @import("cartridge.zig").Cartridge;

const bus_module = @import("../bus.zig");
pub const PageReadFn = bus_module.PageReadFn;
pub const PageWriteFn = bus_module.PageWriteFn;

var g_bus: []u8 = &[_]u8{};
var g_cart: *Cartridge = undefined;
var g_boot: *BootRom = undefined;

// register 0xFF50 determines whether boot ROM addresses are in use since u can turn them on and off
var boot_enabled: bool = true;
var boot_reg_value: u8 = 0;

// setter fo g_bus
pub fn setBus(mem: []u8) void {
    g_bus = mem;
}

// setter for cart and boot variables here
pub fn setCartridgeBoot(cart: *Cartridge, boot: *BootRom) void {
    g_cart = cart;
    g_boot = boot;
}

// return 16 bit bus address
fn calcAddr(mem: []u8, offset: u8) u16 {
    const base_pointer = @intFromPtr(g_bus.ptr);
    const page_pointer = @intFromPtr(mem.ptr);
    const base = page_pointer - base_pointer;
    return @as(u16, @intCast(base + offset));
}

// reads from rom0 addresses
pub fn rom0Read(mem: []u8, offset: u8) u8 {
    const addr = calcAddr(mem, offset);

    // call on boot rom if it is currently enabled
    if(boot_enabled and addr < 0x0100)
    {
        return g_boot.read(addr);
    }

    return g_cart.read(addr);
}

pub fn rom0Write(mem: []u8, offset: u8) u8 {
    const addr = calcAddr(mem, offset);
    g_cart.write(addr);
}






