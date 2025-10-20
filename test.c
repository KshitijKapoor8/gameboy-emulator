
#include <SDL2/SDL.h>
int main() {
  SDL_Init(SDL_INIT_VIDEO);
  SDL_CreateWindow("Test", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640,
                   480, 0);
  SDL_Delay(2000);
  return 0;
}
