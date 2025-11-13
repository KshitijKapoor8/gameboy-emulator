// wram is 8KB of standard RAM
pub fn read(mem: []u8, off: u8) u8 {
    return mem[off];
}
pub fn write(mem: []u8, off: u8, v: u8) void {
    mem[off] = v;
}