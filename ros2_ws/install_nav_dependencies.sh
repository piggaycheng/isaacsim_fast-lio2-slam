#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$workspace_dir/nav_packages"
install_dir="$workspace_dir/nav_install"
declare -a queue=(
  ros-humble-nav2-map-server
  ros-humble-nav2-amcl
  ros-humble-nav2-lifecycle-manager
  ros-humble-nav2-costmap-2d
  ros-humble-nav2-planner
  ros-humble-nav2-controller
  ros-humble-nav2-bt-navigator
  ros-humble-nav2-navfn-planner
  ros-humble-nav2-regulated-pure-pursuit-controller
  ros-humble-pointcloud-to-laserscan
  ros-humble-robot-localization
)
declare -a packages=()
declare -A visited=()

source /opt/ros/humble/setup.bash

while ((${#queue[@]} > 0)); do
  package="${queue[0]}"
  queue=("${queue[@]:1}")

  if [[ -n "${visited[$package]:-}" ]]; then
    continue
  fi
  visited["$package"]=1

  if dpkg-query -W -f='${Status}' "$package" 2>/dev/null |
    grep -q "install ok installed"; then
    continue
  fi
  if ! apt-cache show "$package" >/dev/null 2>&1; then
    echo "Cannot resolve dependency package: $package" >&2
    exit 1
  fi

  packages+=("$package")
  while read -r dependency; do
    if [[ -n "$dependency" && "$dependency" != \<*\> && "$dependency" != *:i386 ]]; then
      queue+=("$dependency")
    fi
  done < <(
    apt-cache depends "$package" |
      sed -n 's/^[[:space:]|]*Depends:[[:space:]]*//p'
  )
done

mkdir -p "$package_dir" "$install_dir"
touch "$package_dir/COLCON_IGNORE" "$install_dir/COLCON_IGNORE"
for package in "${packages[@]}"; do
  echo "Downloading $package"
  (
    cd "$package_dir"
    apt download "$package"
  )
done

find "$package_dir" -maxdepth 1 -name '*.deb' -print0 |
  while IFS= read -r -d '' archive; do
    dpkg-deb -x "$archive" "$install_dir"
  done

# tf2 < 0.25.24 can deadlock the TF listener against a MessageFilter's
# waitForTransform (ros2/geometry2#990), freezing Nav2 TF after minutes.
tf2_fixed_version="0.25.24"
tf2_lib_dir="$install_dir/opt/ros/humble/lib"
tf2_stamp="$tf2_lib_dir/libtf2.so.fixed-$tf2_fixed_version"
installed_tf2="$(dpkg-query -W -f='${Version}' ros-humble-tf2 2>/dev/null || true)"
if dpkg --compare-versions "${installed_tf2:-0}" ge "$tf2_fixed_version"; then
  rm -f "$tf2_lib_dir"/libtf2.so "$tf2_lib_dir"/libtf2.so.fixed-*
  echo "System tf2 $installed_tf2 already contains the deadlock fix"
elif [[ -f "$tf2_stamp" && -f "$tf2_lib_dir/libtf2.so" ]]; then
  echo "Deadlock-fixed tf2 $tf2_fixed_version already installed"
else
  tf2_src="$package_dir/geometry2-$tf2_fixed_version"
  tf2_build="$package_dir/tf2-build"
  if [[ ! -d "$tf2_src" ]]; then
    curl -fsSL "https://github.com/ros2/geometry2/archive/refs/tags/$tf2_fixed_version.tar.gz" |
      tar -xz -C "$package_dir"
  fi
  cmake -S "$tf2_src/tf2" -B "$tf2_build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DCMAKE_INSTALL_PREFIX="$tf2_build/install"
  cmake --build "$tf2_build" --target tf2 -j"$(nproc)"
  mkdir -p "$tf2_lib_dir"
  install -m 0755 "$tf2_build/libtf2.so" "$tf2_lib_dir/libtf2.so"
  touch "$tf2_stamp"
  echo "Installed deadlock-fixed tf2 $tf2_fixed_version in: $tf2_lib_dir"
fi

echo "Navigation dependencies installed in: $install_dir"
