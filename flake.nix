{
  description = "Development shell for ROS 2 Jazzy with Gazebo on NixOS";

  nixConfig = {
    extra-substituters = [
      "https://cache.nixos.org"
      "https://ros.cachix.org"
    ];
    extra-trusted-public-keys = [
      "cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY="
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
  };

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { self, nixpkgs, flake-utils, nix-ros-overlay, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ros = pkgs.rosPackages.jazzy;
      in {
        devShells.default = pkgs.mkShell {
          packages = [
            pkgs.cmake
            pkgs.python3
            pkgs.colcon
            pkgs.vcstool
            pkgs.pkg-config
            (ros.buildEnv {
              paths = [
                ros.ros-core
                ros.ament-cmake-core
                ros.python-cmake-module
                ros.rviz2
                ros.robot-state-publisher
                ros.controller-manager
                ros.joint-state-publisher-gui
                ros.xacro
                ros.ros2launch
                ros.ros_gz_sim
                ros.ros_gz_bridge
                ros.ros_gz_image
                ros.gz_ros2_control
                ros.gz-launch-vendor
              ];
            })
          ];

          shellHook = ''
            export ROS_DISTRO=jazzy
          '';
        };
      });
}