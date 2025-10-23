const std = @import("std");
const windowing = @import("window.zig");

pub fn main() !void {
    try windowing.makeWindow(640, 380);
    return;
}
