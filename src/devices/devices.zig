pub const ReadFn  = *const fn (mem: []u8, off: u8) u8;
pub const WriteFn = *const fn (mem: []u8, off: u8, v: u8) void;
pub const InitFn  = *const fn () void; // device-specific init(cfg) is called by you

pub const Device = struct {
    name: []const u8,
    // each device will have read and write functions
    read: ReadFn,
    write: WriteFn,
    // each device should have an initialization function where it initializes its variables
    init: ?InitFn = null,
};
