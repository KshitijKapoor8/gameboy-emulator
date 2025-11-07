const std = @import("std");
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const expectError = std.testing.expectError;

pub const PAGE_SIZE: u16 = 0x100; // 256
pub const BUS_SIZE: u32 = 0x10000; // 2^16

pub const PageReadFn = *const fn ([]u8, u8) u8;
pub const PageWriteFn = *const fn ([]u8, u8, u8) void;

/// # Represents a fixed sized page that has some backing
pub const Page = struct {
    dirty: bool = false,
    start_address: u16,
    present: bool = false, // potentially if the device is not ready yet
    read_fn: PageReadFn,
    write_fn: PageWriteFn,
};

/// # Represents an abstracted bus divided into fixed sized pages
pub const Bus = struct {
    num_pages: u32,
    pages: [BUS_SIZE / PAGE_SIZE]Page = undefined,
    mem: [BUS_SIZE]u8,
    size: usize,

    /// # Create a new bus
    ///
    /// Creates a default 64KB bus that has page start_address
    /// organized ascending
    ///
    /// # Returns
    ///
    /// An instance of a Bus
    pub fn init() Bus {
        var bus: Bus = .{
            .num_pages = BUS_SIZE / PAGE_SIZE,
            .pages = undefined,
            .mem = .{0} ** BUS_SIZE,
            .size = BUS_SIZE,
        };

        for (&bus.pages, 0..bus.num_pages) |*page, i| {
            page.start_address = @intCast(PAGE_SIZE * i);
        }

        return bus;
    }

    /// # Translate an address to a page
    ///
    /// # Parameters
    /// - `self`: a reference to the struct
    /// - `addr`: a u16 representing the address on the bus to get the page for
    ///
    /// # Returns
    /// A reference to the page that `addr` belongs to, or errors if the page is out of bounds
    pub fn addrToPage(self: *Bus, addr: u16) !*Page {
        const page_number: u16 = addr / PAGE_SIZE;
        if (page_number >= self.num_pages) {
            return error.PageNotFound;
        }

        return &self.pages[page_number];
    }
};

test "Address to page gives correct pages" {
    var bus: Bus = Bus.init();
    bus.pages[0].present = true;

    const first_page: *Page = try bus.addrToPage(0x0);
    try expect(first_page.present);
    try expectEqual(0x0, first_page.start_address);

    const NTH_PAGE: u8 = 127;
    bus.pages[NTH_PAGE].present = true;

    const nth_page: *Page = try bus.addrToPage(NTH_PAGE * PAGE_SIZE);
    try expect(nth_page.present);
    try expectEqual(NTH_PAGE * PAGE_SIZE, nth_page.start_address);

    const MTH_PAGE: u8 = 164;
    var k: u16 = 57;
    bus.pages[MTH_PAGE].present = true;
    bus.pages[MTH_PAGE + 1].present = true;
    const mth_page_plus_k: *Page = try bus.addrToPage(MTH_PAGE * PAGE_SIZE + k);
    try expect(mth_page_plus_k.present);
    try expectEqual(MTH_PAGE * PAGE_SIZE, mth_page_plus_k.start_address);

    k = PAGE_SIZE - 1;
    const mth_page_plus_page_size_minus_one: *Page = try bus.addrToPage(MTH_PAGE * PAGE_SIZE + k);
    try expect(mth_page_plus_page_size_minus_one.present);
    try expectEqual(MTH_PAGE * PAGE_SIZE, mth_page_plus_page_size_minus_one.start_address);

    k = PAGE_SIZE;
    const mth_page_plus_page_size: *Page = try bus.addrToPage(MTH_PAGE * PAGE_SIZE + k);
    try expect(mth_page_plus_page_size.present);
    try expectEqual((MTH_PAGE + 1) * PAGE_SIZE, mth_page_plus_page_size.start_address);
}
