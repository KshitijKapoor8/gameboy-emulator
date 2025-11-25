const PageReadFn = @import("../bus.zig").PageReadFn;
const PageWriteFn = @import("../bus.zig").PageWriteFn;

pub const InitFn = *const fn () void; // device-specific init(cfg) is called by you

pub const Device = struct {
    name: []const u8,
    // each device will have read and write functions
    read: PageReadFn,
    write: PageWriteFn,
    // each device should have an initialization function where it initializes its variables
    init: ?InitFn = null,
};
