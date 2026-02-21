#!/usr/bin/env bash
set -euo pipefail

HTML_FILE="${1:-control_panel.html}"
PORT="${2:-8080}"
IFACE="${3:-wlan0}"

# Wait until NetworkManager reports the interface is connected
while true; do
  state="$(nmcli -t -f DEVICE,STATE dev status | awk -F: -v d="$IFACE" '$1==d{print $2}')"
  if [[ "$state" == "connected" ]]; then
    break
  fi
  sleep 1
done

# Get IPv4 address for the interface (CIDR -> strip /xx)
ip_cidr="$(nmcli -g IP4.ADDRESS dev show "$IFACE" | head -n1 || true)"
ip="${ip_cidr%%/*}"

if [[ -z "${ip}" ]]; then
  echo "No IPv4 address found on ${IFACE} even though it's connected."
  exit 1
fi

# Replace the BASE line in the HTML file
# Matches: const BASE = "http://something:8080";
sed -i -E \
  "s|^(const[[:space:]]+BASE[[:space:]]*=[[:space:]]*\")http://[^\"]*(\";)|\1http://${ip}:${PORT}\2|" \
  "$HTML_FILE"

echo "Updated BASE to http://${ip}:${PORT} in ${HTML_FILE}"
