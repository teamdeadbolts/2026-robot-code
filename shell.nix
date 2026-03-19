{
  pkgs ? import <nixpkgs> { },
}:
let

in
pkgs.mkShell {
  buildInputs = with pkgs; [
    openjdk17
    nixfmt
  ];

  shellHook = ''
    export JAVA_HOME=${pkgs.openjdk17}
  '';
}
