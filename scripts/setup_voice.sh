#!/usr/bin/env bash
# Install voice I/O dependencies inside the running autobee_dev container.
# Run this ONCE inside the container — no image rebuild required.
#
# Usage (from inside autobee_dev):
#   bash /root/scripts/setup_voice.sh
#
# After this script completes, restart the demo with voice:=true:
#   ros2 launch autobee_core demo_tello.launch.py voice:=true

set -euo pipefail

echo "=== Installing audio system packages ==="
apt-get update -qq && apt-get install -y --no-install-recommends \
  portaudio19-dev libsndfile1 \
  mpg123 ffmpeg espeak libespeak-dev \
  alsa-utils

echo "=== Installing Python voice/dotenv packages ==="
pip3 install --no-cache-dir \
  python-dotenv \
  sounddevice

# Whisper is large (~150 MB model weights on first use) — install the package
# now; weights are downloaded lazily on first transcription call.
pip3 install --no-cache-dir openai-whisper

pip3 install --no-cache-dir gtts pyttsx3

echo ""
echo "=== Verifying audio device access ==="
if aplay -l 2>/dev/null | grep -q "card"; then
  echo "Audio card(s) found:"
  aplay -l 2>/dev/null | grep "card" || true
else
  echo "WARNING: no ALSA audio cards detected."
  echo "  - If running via docker-compose, the .:/workspaces/AUTOBEE:ro and"
  echo "    devices: [/dev/snd] entries should expose host audio."
  echo "  - Container is 'privileged: true' so /dev/snd should be accessible."
  echo "  - On Ubuntu 25.10 with PipeWire, try: AUDIODRIVER=pulse python3 -c 'import sounddevice; print(sounddevice.query_devices())'"
fi

echo ""
echo "=== Verifying Python imports ==="
python3 -c "import sounddevice; print('sounddevice OK')" 2>&1 || echo "sounddevice FAILED"
python3 -c "import whisper; print('whisper OK')"         2>&1 || echo "whisper FAILED"
python3 -c "from gtts import gTTS; print('gtts OK')"     2>&1 || echo "gtts FAILED"
python3 -c "import pyttsx3; print('pyttsx3 OK')"         2>&1 || echo "pyttsx3 FAILED"
python3 -c "from dotenv import load_dotenv; print('python-dotenv OK')" 2>&1 || echo "python-dotenv FAILED"

echo ""
echo "=== Voice setup complete. ==="
echo "Run demo with voice:  ros2 launch autobee_core demo_tello.launch.py voice:=true"
