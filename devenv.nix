{ pkgs, lib, config, inputs, ... }:

{
  enterShell = ''
    export PATH="$PATH:$HOME/workspace/uncrustify/uncrustify-uncrustify-0.72.0/build/bin"
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
    qcachegrind
  ];
}
