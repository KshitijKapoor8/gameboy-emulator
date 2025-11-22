const std = @import("std");
var bus = @import("system_bus.zig").g_test_system_bus;

pub fn loadRomFromRomsDir() ![0x100]u8 {
    var cwd = std.fs.cwd();

    var stdin_buffer: [1024]u8 = undefined;
    var stdin = std.fs.File.stdin().reader(&stdin_buffer);

    var dir = try cwd.openDir("roms", .{ .iterate = true });
    defer dir.close();

    const max_roms = 64;
    var rom_names: [max_roms][]const u8 = undefined;
    var rom_count: usize = 0;

    var names_buf: [4096]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&names_buf);
    const allocator = fba.allocator();

    var it = dir.iterate();
    while (try it.next()) |entry| {
        if (entry.kind != .file) continue;

        if (rom_count >= max_roms) {
            return error.TooManyRoms; // optional, but nice to have
        }

        const name_copy = try allocator.dupe(u8, entry.name);
        rom_names[rom_count] = name_copy;
        rom_count += 1;
    }

    std.debug.print("Available ROMs in ./roms:\n", .{});
    var i: usize = 0;
    while (i < rom_count) : (i += 1) {
        std.debug.print("  {d}) {s}\n", .{ i, rom_names[i] });
    }

    std.debug.print("Select a ROM number: ", .{});

    var input_buf: [1]u8 = undefined;
    _ = try stdin.interface.readSliceAll(&input_buf);

    const selection = try std.fmt.parseInt(usize, &input_buf, 10);

    if (selection >= rom_count) {
        return error.InvalidSelection;
    }

    const rom_name = rom_names[selection];

    const full_path = try std.fs.path.join(allocator, &.{ "roms", rom_name });
    var file = try cwd.openFile(full_path, .{ .mode = .read_only });
    defer file.close();

    var rom_buf: [0x100]u8 = undefined;
    const n = try file.read(&rom_buf);

    std.debug.print("Read {d} bytes", .{n});

    return rom_buf;
}

/// Reads exactly 0x100 (256) bytes from a file in the current directory.
/// Errors if fewer bytes are available.
///
/// `file_path` is relative to cwd(), e.g. "roms/dmg_boot.bin".
pub fn loadFixed256Rom(file_path: []const u8) ![0x100]u8 {
    var cwd = std.fs.cwd();
    var file = try cwd.openFile(file_path, .{ .mode = .read_only });
    defer file.close();

    var buf: [0x100]u8 = undefined;

    const n = try file.readAll(&buf);
    if (n != buf.len)
        return error.FileTooSmall;

    return buf;
}
