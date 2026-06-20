"""Whisper STT node — microphone to /user_intent.

Continuous-listen pipeline with energy-based VAD:
    sounddevice mic stream  ->  ring buffer
                            ->  detect speech onset (RMS over threshold)
                            ->  buffer until silence_ms of quiet
                            ->  whisper transcribes the captured chunk
                            ->  publish std_msgs/String to /user_intent

Dependencies (install in container):
    pip install sounddevice numpy openai-whisper
    apt install portaudio19-dev libsndfile1

Audio device must be passed into the container; see autobee_voice/README.md
for the docker run flags.

Parameters:
    model            "tiny" | "base" | "small" | "medium" (whisper size)
    language         language code passed to whisper (e.g. "en")
    sample_rate      mic sample rate (Hz)
    rms_threshold    speech-onset RMS threshold (float, sounddevice units)
    silence_ms       trailing silence before transcription fires
    max_utterance_s  hard cap on a single recorded chunk
    device           sounddevice input device index/name (None = default)

Falls back gracefully:
    - no whisper installed     -> log fatal, do not crash
    - no audio device          -> log fatal, do not crash
    - empty/unintelligible audio -> log warn, skip publish
"""

import collections
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WhisperSTTNode(Node):
    def __init__(self):
        super().__init__("whisper_stt_node")

        self.declare_parameter("model", "base")
        self.declare_parameter("language", "en")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("rms_threshold", 0.015)
        self.declare_parameter("silence_ms", 800)
        self.declare_parameter("max_utterance_s", 12.0)
        self.declare_parameter("device", "")
        self.declare_parameter("min_speech_ms", 300)

        self.model_name = str(self.get_parameter("model").value)
        self.language = str(self.get_parameter("language").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.rms_threshold = float(self.get_parameter("rms_threshold").value)
        self.silence_ms = int(self.get_parameter("silence_ms").value)
        self.max_utterance_s = float(self.get_parameter("max_utterance_s").value)
        self.min_speech_ms = int(self.get_parameter("min_speech_ms").value)
        device_param = str(self.get_parameter("device").value).strip()
        self.device = device_param if device_param else None

        self.intent_pub = self.create_publisher(String, "/user_intent", 10)

        self._np = None
        self._sd = None
        self._whisper_model = None
        self._stop = threading.Event()
        self._audio_thread = None

        self._init_audio_and_whisper()

        if self._whisper_model is not None and self._sd is not None:
            self._audio_thread = threading.Thread(target=self._listen_loop, daemon=True)
            self._audio_thread.start()
            self.get_logger().info(
                f"whisper_stt_node up (model={self.model_name}, lang={self.language}, "
                f"sr={self.sample_rate}Hz, threshold={self.rms_threshold}); "
                f"speak into the mic to publish on /user_intent"
            )

    def _init_audio_and_whisper(self):
        try:
            import numpy as np
            self._np = np
        except ImportError:
            self.get_logger().fatal("numpy not installed -- pip install numpy")
            return

        try:
            import sounddevice as sd
            self._sd = sd
        except (ImportError, OSError) as e:
            self.get_logger().fatal(
                f"sounddevice unavailable ({e}). Install: pip install sounddevice; "
                "apt install portaudio19-dev. Audio device must be passed into the "
                "container (--device /dev/snd, --group-add audio)."
            )
            return

        try:
            import whisper
            self.get_logger().info(f"loading whisper model {self.model_name!r} (cpu)...")
            self._whisper_model = whisper.load_model(self.model_name, device="cpu")
            self.get_logger().info("whisper model loaded")
        except ImportError:
            self.get_logger().fatal(
                "openai-whisper not installed -- pip install openai-whisper"
            )
        except Exception as e:
            self.get_logger().fatal(f"whisper load failed: {e}")

    def _listen_loop(self):
        sd = self._sd
        np = self._np
        block_ms = 30
        block_samples = int(self.sample_rate * block_ms / 1000)
        silence_blocks_required = max(1, int(self.silence_ms / block_ms))
        max_blocks = int(self.max_utterance_s * 1000 / block_ms)    
        preroll_blocks = int(300 / block_ms)
        preroll = collections.deque(maxlen=preroll_blocks)

        try:
            stream = sd.InputStream(
                samplerate=self.sample_rate,
                channels=1,
                blocksize=block_samples,
                dtype="float32",
                device=self.device,
            )
        except Exception as e:
            self.get_logger().fatal(f"could not open mic stream: {e}")
            return

        with stream:
            buffered = []
            silent_blocks = 0
            recording = False
            start_block = 0

            while not self._stop.is_set():
                try:
                    block, _overflow = stream.read(block_samples)
                except Exception as e:
                    self.get_logger().error(f"mic read error: {e}")
                    time.sleep(0.5)
                    continue
                samples = block[:, 0] if block.ndim > 1 else block
                rms = float(np.sqrt(np.mean(samples * samples) + 1e-12))
                is_voiced = rms >= self.rms_threshold

                if not recording:
                    preroll.append(samples.copy())
                    if is_voiced:
                        recording = True
                        buffered = list(preroll) + [samples.copy()]
                        start_block = 1 + len(preroll)
                        silent_blocks = 0
                else:
                    buffered.append(samples.copy())
                    if is_voiced:
                        silent_blocks = 0
                    else:
                        silent_blocks += 1

                    if silent_blocks >= silence_blocks_required or len(buffered) >= max_blocks:
                        speech_blocks = len(buffered) - silent_blocks
                        speech_ms = speech_blocks * block_ms
                        if speech_ms >= self.min_speech_ms:
                            audio = np.concatenate(buffered).astype(np.float32)
                            self._transcribe_and_publish(audio)
                        buffered = []
                        recording = False
                        silent_blocks = 0

    def _transcribe_and_publish(self, audio):
        try:
            result = self._whisper_model.transcribe(
                audio,
                language=self.language,
                fp16=False,
            )
        except Exception as e:
            self.get_logger().error(f"whisper transcribe failed: {e}")
            return

        text = (result.get("text") or "").strip()
        if not text:
            return

        if len(text) < 3:
            self.get_logger().warn(f"discarding short transcription: {text!r}")
            return

        self.get_logger().info(f"heard: {text!r}")
        msg = String()
        msg.data = text
        self.intent_pub.publish(msg)

    def stop(self):
        self._stop.set()


def main(args=None):
    rclpy.init(args=args)
    node = WhisperSTTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
