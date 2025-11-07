const std = @import("std");
const Bus = @import("bus.zig").Bus;
pub const PAGE_SIZE: u16 = @import("bus.zig").PAGE_SIZE;
pub const BUS_SIZE: u32 = @import("bus.zig").BUS_SIZE;
pub const Page = @import("bus.zig").Page;
pub const PageReadFn = @import("bus.zig").PageReadFn;
pub const PageWriteFn = @import("bus.zig").PageWriteFn;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const expectError = std.testing.expectError;

/// A mapping structure to define read and write callbacks
pub const Mapping = struct {
    start: u16,
    end: u16, // inclusive
    read: PageReadFn,
    write: PageWriteFn,
};

/// The system bus that corresponds to memory and devices
pub const SystemBus = struct {
    bus: *Bus,
    mappings: []const Mapping,

    /// # Initialize the system bus
    ///
    /// This function sets up the pages backing the system bus based on a list of
    /// non-overlapping mappings
    ///
    /// # Parameters
    /// - `self`: a reference to the system bus struct
    /// - `mappings`: an array of mappings, where each mapping specifies a start and end
    ///    address as well as read and write function callbacks (for things like MMIO). Note that
    ///    it is assumed that the mappings are page-aligned
    ///
    /// # Returns
    /// Can return an OutOfMemory error if the mappings specified would exceed the size of the bus,
    /// returns nothing otherwise
    pub fn init(self: *SystemBus, mappings: []const Mapping) !void {
        self.mappings = mappings;
        var size: u32 = 0;

        for (self.mappings) |mapping| {
            size += (@as(u32, mapping.end) - @as(u32, mapping.start)) + 1;
        }

        if (size > BUS_SIZE) {
            return error.OutOfMemory;
        }

        for (self.mappings) |mapping| {
            var addr: u32 = mapping.start;
            const end: u32 = mapping.end;

            while (addr <= end) : (addr += PAGE_SIZE) {
                var page: *Page = try self.bus.addrToPage(@intCast(addr));
                page.read_fn = mapping.read;
                page.write_fn = mapping.write;
                page.present = true;
            }

            size += (@as(u32, mapping.end) - @as(u32, mapping.start)) + 1;
        }
    }

    /// # Read a single byte at a bus address
    ///
    /// This function will first get a page from the address and then
    /// dispatch to the specific read callback as specified by the page
    ///
    /// # Parameters
    /// - `self`: a reference to the SystemBus struct
    /// - `addr`: a u16 that specifies the bus address to read
    ///
    /// # Returns
    /// Error if the page cannot be found or the byte otherwise
    pub fn read(self: *SystemBus, addr: u16) !u8 {
        const page = try self.bus.addrToPage(addr);
        if (!page.present) return error.PageNotFound;

        const start: usize = page.start_address;
        const slice = self.bus.mem[start .. start + PAGE_SIZE];

        return page.read_fn(slice, @intCast(addr % PAGE_SIZE));
    }

    /// # Write a single byte to a bus address
    ///
    /// This function will first get a page from the address and then
    /// dispatch to the specific write callback as specified by the page
    ///
    /// # Parameters
    /// - `self`: a reference to the SystemBus struct
    /// - `addr`: a u16 that specifies the bus address to write to
    /// - `val`: a u8 that specifies the value to write
    ///
    /// # Returns
    /// Error if the page cannot be found or nothing otherwise
    pub fn write(self: *SystemBus, addr: u16, val: u8) !void {
        const page = try self.bus.addrToPage(addr);
        if (!page.present) return error.PageNotFound;

        const start: usize = page.start_address;
        const slice = self.bus.mem[start .. start + PAGE_SIZE];

        page.write_fn(slice, @intCast(addr % PAGE_SIZE), val);
    }
};

fn pageReadBasic(mem: []u8, offset: u8) u8 {
    return mem[offset];
}

fn pageWriteBasic(mem: []u8, offset: u8, byte: u8) void {
    mem[offset] = byte;
}

const test_basic_mappings = [1]Mapping{
    .{ .start = 0x0000, .end = 0xFFFF, .read = pageReadBasic, .write = pageWriteBasic },
};

var bus = Bus.init();

fn makeSystemBusFull(mappings: []const Mapping) !struct { bus: Bus, sysbus: SystemBus } {
    var sysbus = SystemBus{ .bus = &bus, .mappings = &.{} };
    try sysbus.init(mappings);
    return .{ .bus = bus, .sysbus = sysbus };
}

test "Create SystemBus" {
    const sysbus: SystemBus = (try makeSystemBusFull(&test_basic_mappings)).sysbus;

    try expect(sysbus.bus.pages[0].read_fn == pageReadBasic);
    try expect(sysbus.bus.pages[0].write_fn == pageWriteBasic);
}

test "Bus write, then read" {
    var sysbus: SystemBus = (try makeSystemBusFull(&test_basic_mappings)).sysbus;

    const value = 0x42;
    try sysbus.write(0x1234, value);
    try expect(try sysbus.read(0x1234) == value);
}

test "Write at end of page, read back" {
    var sysbus: SystemBus = (try makeSystemBusFull(&test_basic_mappings)).sysbus;

    const addr: u16 = 0x00FF;
    const val: u8 = 0xAA;
    try sysbus.write(addr, val);
    try expect(try sysbus.read(addr) == val);
}

fn pageReadAlt(_: []u8, _: u8) u8 {
    return 0x99;
}
fn pageWriteAlt(_: []u8, _: u8, _: u8) void {
    return;
}

test "Different mappings apply to different pages" {
    const mappings = [_]Mapping{
        .{ .start = 0x0000, .end = 0x00FF, .read = pageReadBasic, .write = pageWriteBasic },
        .{ .start = 0x0100, .end = 0x01FF, .read = pageReadAlt, .write = pageWriteAlt },
    };
    var sysbus: SystemBus = (try makeSystemBusFull(&mappings)).sysbus;

    // first page should use basic
    try sysbus.write(0x000A, 0x11);
    try expectEqual(0x11, try sysbus.read(0x000A));

    // second page should use alt (always returns 0x99)
    try expectEqual(0x99, try sysbus.read(0x010A));
}

test "Mapped pages have present=true" {
    var local_bus = Bus.init();
    const partial = [_]Mapping{
        .{ .start = 0x0000, .end = 0x03FF, .read = pageReadBasic, .write = pageWriteBasic }, // 1KB -> first 4 pages
    };
    var sys = SystemBus{ .bus = &local_bus, .mappings = &.{} };
    try sys.init(&partial);

    // pages 0..3 should be present, while 4 should not
    try expect(local_bus.pages[0].present);
    try expect(local_bus.pages[1].present);
    try expect(local_bus.pages[2].present);
    try expect(local_bus.pages[3].present);
    try expect(!local_bus.pages[4].present);
}

test "Unmapped address returns PageNotFound" {
    var local_bus = Bus.init();
    const partial = [_]Mapping{
        .{ .start = 0x0000, .end = 0x0FFF, .read = pageReadBasic, .write = pageWriteBasic },
    };

    var sysbus = SystemBus{ .bus = &local_bus, .mappings = &.{} };
    try sysbus.init(&partial);

    // no backing, fail
    try expectError(error.PageNotFound, sysbus.read(0x9000));
}

test "Init fails when mappings exceed bus size" {
    // full bus + one extra byte
    const bad = [_]Mapping{
        .{ .start = 0x0000, .end = 0xFFFF, .read = pageReadBasic, .write = pageWriteBasic },

        .{ .start = 0x10000 - 1, .end = 0x10000 - 1, .read = pageReadBasic, .write = pageWriteBasic },
    };

    try expectError(error.OutOfMemory, makeSystemBusFull(&bad));
}
