#!/usr/bin/env bash
# Fetch the ORB-SLAM3 vocabulary file (ORBvoc.txt, ~145 MB).
#
# Stored alongside the ORB-SLAM3 source build at:
#   /opt/autobee/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt
#
# Idempotent: skips download if the file already exists.

set -euo pipefail

ORB_DIR=/opt/autobee/third_party/ORB_SLAM3
VOCAB="${ORB_DIR}/Vocabulary/ORBvoc.txt"
TARBALL="${ORB_DIR}/Vocabulary/ORBvoc.txt.tar.gz"

if [ ! -d "$ORB_DIR" ]; then
  echo "ERROR: ORB-SLAM3 not found at $ORB_DIR; run scripts/build_orbslam3.sh first" >&2
  exit 1
fi

if [ -f "$VOCAB" ]; then
  echo "ORBvoc.txt already present (size: $(du -h "$VOCAB" | cut -f1))"
  exit 0
fi

# The tarball ships in the ORB-SLAM3 repo; if it's there, just untar.
if [ -f "$TARBALL" ]; then
  echo "Extracting $TARBALL ..."
  tar -xzf "$TARBALL" -C "${ORB_DIR}/Vocabulary"
  echo "Done."
  exit 0
fi

echo "ORBvoc.txt.tar.gz not found at $TARBALL." >&2
echo "Re-run scripts/build_orbslam3.sh -- it clones the repo with the vocab tarball included." >&2
exit 1
