const std = @import("std");
const expect = std.testing.expect;
const windowing = @import("window.zig");

comptime {
    _ = @import("bus.zig");
}

pub fn main() !void {
    try windowing.makeWindow(640, 380);
    return;
}

test "Basic test" {
    try expect(1 == 1);
}
