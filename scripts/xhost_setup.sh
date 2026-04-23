#!/usr/bin/env bash
# Run on the host before `docker compose up` to allow X11 forwarding into the container.
# Safe because we scope access to local containers only.
set -euo pipefail

touch /tmp/.docker.xauth
xhost +local:docker >/dev/null

echo "X11 access granted to local Docker containers."
echo "Now run: docker compose run --rm autobee"
