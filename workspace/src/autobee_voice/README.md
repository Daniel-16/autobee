# autobee_voice

Voice I/O for AutoBee.

- **`whisper_stt_node`**: continuous-listen mic → `/user_intent` (Whisper, CPU,
  energy-VAD).
- **`tts_node`**: speaks `/mission_report` (and optionally `/mission_status`
  events) via gTTS or pyttsx3.

## Container audio passthrough

Voice nodes need access to the host audio devices and a network out (for
gTTS). When (re)starting the dev container:

```bash
docker run --rm -it \
  --network=host \
  --ipc=host \
  --device /dev/snd \
  --group-add audio \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/daniel-toba/Documents/AUTOBEE:/workspaces/AUTOBEE \
  autobee:humble bash
```

> The persistent `autobee_dev` container does NOT have `--device /dev/snd`.
> Voice will not work in that container without re-creating it. Either start
> a fresh container with the flags above, OR add `--device /dev/snd
> --group-add audio` to your `docker-compose.yml` and recreate.

## One-time install (inside container)

```bash
pip install --no-cache-dir openai-whisper sounddevice numpy gtts pyttsx3
apt-get update && apt-get install -y portaudio19-dev libsndfile1 mpg123 espeak
```

`openai-whisper` is large (~1 GB cache) on first model download. `tiny` model
is ~75 MB and fine for command vocabulary; `base` is the recommended default.

## Smoke tests

```bash
# Speak something to the mic, watch /user_intent fire:
ros2 run autobee_voice whisper_stt_node
ros2 topic echo /user_intent

# Force the TTS to say something:
ros2 run autobee_voice tts_node
ros2 topic pub --once /tts_say std_msgs/msg/String '{data: "Hello operator."}'
```

## Wired into the full demo

`demo.launch.py` accepts a `voice:=true` argument. Without it, voice is OFF
and the system runs unchanged from before.

```bash
ros2 launch autobee_core demo.launch.py voice:=true
```

## Failure modes & graceful degradation

| Failure | Behaviour |
|---|---|
| `sounddevice` import error | STT node logs fatal, stays alive (no publish) |
| no `/dev/snd` in container | mic stream open fails; STT logs fatal, stays alive |
| `openai-whisper` not installed | STT node logs fatal, stays alive |
| `gtts` not installed or no internet | TTS auto-falls-back to `pyttsx3` (offline) |
| no audio player on PATH | gTTS backend disabled; falls back to pyttsx3 |
| pyttsx3 also unavailable | TTS falls through to `print` backend (logs the text) |

In every degraded path, the rest of the autonomy stack continues — voice is
a UX layer, not a control-path dependency.
