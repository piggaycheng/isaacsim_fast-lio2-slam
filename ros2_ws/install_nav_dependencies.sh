#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$workspace_dir/nav_packages"
install_dir="$workspace_dir/nav_install"
declare -a queue=(
  ros-humble-nav2-map-server
  ros-humble-nav2-amcl
  ros-humble-nav2-lifecycle-manager
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

echo "Navigation dependencies installed in: $install_dir"
