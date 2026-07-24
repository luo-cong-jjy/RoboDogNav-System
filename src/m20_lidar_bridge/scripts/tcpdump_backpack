#!/usr/bin/env bash
set -eo pipefail

IFACE="${1:-enp2s0}"

exec timeout 10 sudo tcpdump -ni "$IFACE" \
  'udp and (host 224.10.10.201 or host 224.10.10.202) and (port 6691 or port 7781 or port 6692 or port 7782)' \
  -c 30
