const std = @import("std");

pub const BootRom = struct {
    // bytes in boot ROM that can be read
    data: []const u8,

    pub fn init(data: []const u8) BootRom {
        return .{ .data = data };
    }

    pub fn read(self: *const BootRom, addr: u16) u8 {
        if (addr < self.data.len) {
            return self.data[addr];
        }
        return 0xFF;
    }
};
