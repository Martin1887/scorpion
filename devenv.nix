{ pkgs, lib, config, inputs, ... }:
let
  pkgs-unstable = import inputs.nixpkgs-unstable { system = pkgs.stdenv.system; };
in
{
  enterShell = ''
    export PATH="$PATH:$HOME/workspace/uncrustify/uncrustify-uncrustify-0.72.0/build/bin"
    export soplex_DIR="/nix/store/3lj32g14bydv3hg567kz26dp32xbwjs6-soplex-712"
  '';

  languages.python = {
    enable = true;
    poetry.enable = false;
    uv.enable = false;
    venv.enable = true;
  };

  packages = with pkgs; [
    ruff
    cmake
    clang-tools
    valgrind
    pkgs-unstable.gmp
    pkgs-unstable.soplex
    qcachegrind
  ];
}
