const c = @cImport({
    @cInclude("SDL2/SDL.h");
});

pub fn main() !void {
    if (c.SDL_Init(c.SDL_INIT_VIDEO) != 0) return;
    defer c.SDL_Quit();

    const window = c.SDL_CreateWindow(
        "Hello Game Boy!",
        c.SDL_WINDOWPOS_CENTERED,
        c.SDL_WINDOWPOS_CENTERED,
        640,
        480,
        c.SDL_WINDOW_SHOWN,
    ) orelse return;
    defer c.SDL_DestroyWindow(window);

    c.SDL_Delay(2000); // show window for 2 seconds then exit
}
