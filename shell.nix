{
  pkgs ? import <nixpkgs> { },
}:
let

in
pkgs.mkShell {
  buildInputs = with pkgs; [
    openjdk17
    nixfmt
    visualvm
    libGL
    libglvnd
  ];

  shellHook = ''
    export JAVA_HOME=${pkgs.openjdk17}
  '';
}
