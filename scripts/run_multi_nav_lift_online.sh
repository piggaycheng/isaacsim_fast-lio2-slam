#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Shared per-floor costmap zones are on by default; pass --no-filter-editor to disable.
exec "$project_dir/scripts/run_multi_nav_online.sh" --elevator-lift "${ELEVATOR_LIFT:-lift_1}" \
  --filter-editor "$@"
