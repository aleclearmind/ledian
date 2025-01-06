{
  inputs.nixpkgs.url = "nixpkgs";
  inputs.flake-utils.url = "github:numtide/flake-utils";

  outputs =
    { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs { system = system; };
    in
    rec {
      formatter = pkgs.nixfmt-rfc-style;
      packages.libvterm = pkgs.stdenv.mkDerivation {
        pname = "libvterm";
        version = "0.1.0";
        src = pkgs.fetchbzr {
          url = "lp:libvterm";
          rev = "844";
          hash = "sha256-k54YeKZkkEd23YlVPj8nkw5MRObHYngNHK1aQtoZn4s=";
        };
        buildInputs = [
          pkgs.gcc
          pkgs.gnumake
          pkgs.perl
          pkgs.libtool
        ];

        buildPhase = "make";
        installFlags = [ "PREFIX=$(out)" ];
      };
      packages.pangoterm = pkgs.stdenv.mkDerivation {
        pname = "pangoterm";
        version = "0.1.0";
        src = ./.;
        buildInputs = [
          pkgs.gcc
          pkgs.gnumake
          pkgs.libtool
          pkgs.glib
          pkgs.pkg-config
          pkgs.cairo
          pkgs.gtk2
          packages.libvterm
        ];

        buildPhase = "make";
        installFlags = [ "PREFIX=$(out)" ];
      };
      packages.grid = pkgs.mkShell {
        buildInputs = [
          pkgs.blender
          pkgs.python311
          pkgs.python311Packages.mapbox-earcut
          pkgs.python311Packages.manifold3d
          pkgs.python311Packages.black
          pkgs.python311Packages.trimesh
          pkgs.python311Packages.pyglet
          pkgs.python311Packages.ipython
          pkgs.python311Packages.scipy
          pkgs.python311Packages.shapely
          pkgs.python311Packages.mypy
          pkgs.python311Packages.ezdxf
          pkgs.python311Packages.svgwrite
          pkgs.python311Packages.numpy
        ];
      };
    });
}
