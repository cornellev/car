{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };
  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "ros2_ws";
          packages = [
            pkgs.colcon
            pkgs.python3
            pkgs.python3Packages.fastapi
            pkgs.python3Packages.uvicorn
            pkgs.python3Packages.docker
            # ... other non-ROS packages
            (
              with pkgs.rosPackages.humble;
              buildEnv {
                paths = [
                  ros-core
                  ros2bag
                  rosbag2-storage-default-plugins
                  # ... other ROS packages
                ];
              }
            )
            
          ];
          shellHook = ''
            source <(python3 install/_local_setup_util_sh.py zsh)
            export COLCON_CURRENT_PREFIX=$(pwd)/install
          '';
        };
      }
    );
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
