const c = @cImport({
    @cInclude("SDL2/SDL.h");
});

const std = @import("std");

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
    );
    defer c.SDL_DestroyWindow(window);

    if (window == null) {
        std.debug.print("SDL_CreateWindow failed: {s}\n", .{c.SDL_GetError()});
        return;
    }

    const screenSurface = c.SDL_GetWindowSurface(window);
    _ = c.SDL_FillRect(screenSurface, null, c.SDL_MapRGB(screenSurface.*.format, 0xFF, 0xFF, 0xFF));
    _ = c.SDL_UpdateWindowSurface(window);
    c.SDL_Delay(2000);
    c.SDL_DestroyWindow(window);
    c.SDL_Quit();
}
