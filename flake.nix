{
  inputs.nixpkgs.url = "nixpkgs";

  outputs =
    { self, nixpkgs }:
    let
      pkgs = import nixpkgs { system = "x86_64-linux"; };
      libvterm = pkgs.stdenv.mkDerivation {
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
      pangoterm = pkgs.stdenv.mkDerivation {
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
          libvterm
        ];

        buildPhase = "make";
        installFlags = [ "PREFIX=$(out)" ];
      };
      grid = pkgs.mkShell {
        buildInputs = [
          pkgs.blender
          pkgs.python312
          pkgs.python312Packages.mapbox-earcut
          pkgs.python312Packages.manifold3d
          pkgs.python312Packages.black
          pkgs.python312Packages.trimesh
          pkgs.python312Packages.pyglet
          pkgs.python312Packages.ipython
          pkgs.python312Packages.scipy
          pkgs.python312Packages.shapely
          pkgs.python312Packages.mypy
          pkgs.python312Packages.ezdxf
          pkgs.python312Packages.svgwrite
          pkgs.python312Packages.numpy
        ];
      };
    in
    {
      formatter.x86_64-linux = nixpkgs.legacyPackages.x86_64-linux.nixfmt-rfc-style;
      packages.x86_64-linux.libvterm = libvterm;
      packages.x86_64-linux.pangoterm = pangoterm;
      packages.x86_64-linux.grid = grid;
    };
}
