const std = @import("std");
const c = @cImport({
    @cInclude("SDL2/SDL.h");
});

pub fn makeWindow(width: u16, height: u16) !void {
    if (c.SDL_Init(c.SDL_INIT_VIDEO) != 0) return;
    defer c.SDL_Quit();

    const window = c.SDL_CreateWindow(
        "Hello Game Boy!",
        c.SDL_WINDOWPOS_CENTERED,
        c.SDL_WINDOWPOS_CENTERED,
        width,
        height,
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

    var event: c.SDL_Event = undefined;
    var running: bool = true;
    while (running) {
        while (c.SDL_PollEvent(&event) != 0) {
            if (event.type == c.SDL_QUIT) running = false;
        }

        c.SDL_Delay(10);
    }
}
