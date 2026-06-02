{
  description = "
    A ros2 flake for nixos
    https://github.com/lopsided98/nix-ros-overlay
  ";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
  };
  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in {
        devShells.default = pkgs.mkShell {
          name = "ROS2 Jazzy Workspace";
          packages = [
            # Build and package management
            pkgs.colcon
            pkgs.cmake
            pkgs.python3
            
            # Python libraries for development
            pkgs.python3Packages.numpy
            pkgs.python3Packages.opencv4
            pkgs.python3Packages.pyyaml
            
            # Development tools
            pkgs.git
            pkgs.gcc
            pkgs.pkg-config
            pkgs.libxkbcommon

            pkgs.python3Packages.argcomplete
            pkgs.python3Packages.setuptools
            
            (with pkgs.rosPackages.jazzy; buildEnv {
              paths = [
                # ROS2 core
                ros-core
                
                # Visualization and tools
                rviz2
                robot-state-publisher
                joint-state-publisher-gui
                xacro
                
                # Control and simulation
                ros2-control
                controller-manager
                ros-gz
                
                # Build tools and generators
                ament-cmake               
                ament-cmake-core
                #ament-python
                python-cmake-module
                rosidl-default-generators
                rosidl-default-runtime
                
                # Common ROS2 packages
                std-msgs
                std-srvs
                sensor-msgs
                geometry-msgs
                
                # Testing and linting
                ament-lint-auto
                ament-flake8
                ament-pep257
              ];
            })
          ];
          
          shellHook = ''
            export ROS_DISTRO=jazzy

            source ${pkgs.rosPackages.jazzy.ros-core}/setup.bash
            [ -f ./install/setup.bash ] && source ./install/setup.bash

            source ${pkgs.bash-completion}/etc/profile.d/bash_completion.sh

            # IMPORTANT: explicit ROS2 argcomplete registration
            if command -v register-python-argcomplete >/dev/null 2>&1; then
              eval "$(register-python-argcomplete ros2)"
            fi
          '';
        };
      });
  nixConfig = {
    # extra-substituters = [ "https://ros.cachix.org" ];
    # extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
    # extra-substituters = https://attic.iid.ciirc.cvut.cz/ros
    # extra-trusted-public-keys = ros:JR95vUYsShSqfA1VTYoFt1Nz6uXasm5QrcOsGry9f6Q=
  };
}