#!/usr/bin/env bash
set -eo pipefail

# run_multi_nav.sh with the outbound connections on by default: the camera list is
# published to an MQTT broker (and the ceiling cameras are offered over RTSP). Without it, run_multi_nav.sh opens no RTSP/MQTT.
# All run_multi_nav.sh options work; an explicit --mqtt-* option overrides the defaults.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mqtt_host="${MQTT_HOST:-localhost}"
mqtt_port="${MQTT_PORT:-1883}"
mqtt_topic="${MQTT_TOPIC:-slam/cameras}"

for arg in "$@"; do
  case "$arg" in
    -h|--help)
      cat <<EOF
Usage: ./scripts/run_multi_nav_online.sh [run_multi_nav.sh OPTIONS]

Same as run_multi_nav.sh, but publishes the camera list to MQTT and offers the
ceiling cameras over RTSP by default.
Defaults (override with env vars or --mqtt-* options):
  MQTT_HOST=$mqtt_host  MQTT_PORT=$mqtt_port  MQTT_TOPIC=$mqtt_topic
The broker may be down at start; the client keeps retrying in the background.
Use --camera [ROBOT=]CAMERA to also offer robot cameras over RTSP.

EOF
      exec "$script_dir/run_multi_nav.sh" --help
      ;;
  esac
done

defaults=(--ceiling-cameras)
for option in host port topic; do
  given=false
  for arg in "$@"; do
    if [[ "$arg" == "--mqtt-$option" ]]; then given=true; fi
  done
  if [[ "$given" == false ]]; then
    variable="mqtt_$option"
    defaults+=("--mqtt-$option" "${!variable}")
  fi
done

exec "$script_dir/run_multi_nav.sh" "${defaults[@]}" "$@"
