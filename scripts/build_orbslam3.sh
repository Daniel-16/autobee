#!/usr/bin/env bash
set -euo pipefail

# ORB-SLAM3 build script. Run inside the autobee Docker container.
# Splits the painful source build from the Dockerfile so failures are recoverable
# without rebuilding the whole image.

ORB_DIR=/opt/autobee/third_party/ORB_SLAM3

if [ ! -d "$ORB_DIR" ]; then
    mkdir -p /opt/autobee/third_party
    cd /opt/autobee/third_party
    git config --global http.postBuffer 524288000
    git config --global http.version HTTP/1.1
    git config --global http.lowSpeedLimit 1000
    git config --global http.lowSpeedTime 300
    for i in 1 2 3 4 5; do
        echo "Clone attempt $i..."
        if git clone --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git; then
            break
        fi
        rm -rf ORB_SLAM3
        sleep 5
    done
fi

cd "$ORB_DIR"

# Bump ORB-SLAM3 from C++11 to C++17 to match Pangolin's sigslot header requirements.
# Idempotent: only patches if c++11 is still set.
for cml in CMakeLists.txt Examples/ROS/ORB_SLAM3/CMakeLists.txt Thirdparty/g2o/CMakeLists.txt; do
    if [ -f "$cml" ]; then
        sed -i 's/-std=c++11/-std=c++14/g; s/CMAKE_CXX_STANDARD 11/CMAKE_CXX_STANDARD 14/g' "$cml"
    fi
done

chmod +x build.sh
./build.sh

echo ""
echo "ORB-SLAM3 built. Binaries in $ORB_DIR/Examples"
echo "Next: build the ros2-ORB_SLAM3 wrapper inside /root/autobee_ws/src"
