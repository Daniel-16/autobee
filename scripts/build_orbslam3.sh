#!/usr/bin/env bash
set -euo pipefail

# ORB-SLAM3 build script. Run inside the autobee Docker container.
# Splits the painful source build from the Dockerfile so failures are recoverable
# without rebuilding the whole image.

ORB_DIR=/opt/autobee/third_party/ORB_SLAM3

if [ ! -d "$ORB_DIR" ]; then
    mkdir -p /opt/autobee/third_party
    cd /opt/autobee/third_party
    git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
fi

cd "$ORB_DIR"
chmod +x build.sh
./build.sh

echo ""
echo "ORB-SLAM3 built. Binaries in $ORB_DIR/Examples"
echo "Next: build the ros2-ORB_SLAM3 wrapper inside /root/autobee_ws/src"
