{
  description = "User-level network FS";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  outputs =
    { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        name = "gameboy";
        packages = with pkgs; [
          zig
          zls
          pkg-config
          cmake
          gcc
          SDL2
        ];
        shellHook = ''echo "dev shell: sdl2" '';
      };
    };
}
