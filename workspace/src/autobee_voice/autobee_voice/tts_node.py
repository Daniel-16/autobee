"""TTS node — /mission_report (and optional /mission_status) to spoken audio.

Subscribes:
    /mission_report     std_msgs/String -- always spoken
    /mission_status     std_msgs/String (JSON) -- optional, only "started"
                        and terminal events spoken (off by default)
    /tts_say            std_msgs/String -- ad-hoc "speak this now" channel

Publishes:
    /tts_speaking       std_msgs/String ("started" / "finished" + text)

Backends, in order of preference (selected by `backend` parameter or
auto-fallback at startup):
    - "gtts"     : Google TTS (online, best quality), played via mpg123
    - "pyttsx3"  : offline (espeak under the hood), no internet required
    - "print"    : last-resort no-op that just logs the text

Falls back gracefully:
    - no internet            -> auto-fallback gTTS -> pyttsx3
    - no audio output device -> log fatal once, keep subscribing (no-op speak)
    - long reports           -> truncated to max_chars (default 600)
"""

import json
import os
import queue
import shutil
import subprocess
import tempfile
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TTSNode(Node):
    def __init__(self):
        super().__init__("tts_node")

        self.declare_parameter("backend", "auto")
        self.declare_parameter("language", "en")
        self.declare_parameter("speak_status", False)
        self.declare_parameter("max_chars", 600)
        self.declare_parameter("audio_player", "")
        self.declare_parameter("speak_intro", True)

        backend_param = str(self.get_parameter("backend").value)
        self.language = str(self.get_parameter("language").value)
        self.speak_status = bool(self.get_parameter("speak_status").value)
        self.max_chars = int(self.get_parameter("max_chars").value)
        player = str(self.get_parameter("audio_player").value).strip()
        self._player = player or self._autodetect_player()
        self._speak_intro = bool(self.get_parameter("speak_intro").value)

        self.speaking_pub = self.create_publisher(String, "/tts_speaking", 10)
        self.create_subscription(String, "/mission_report", self._on_report, 10)
        self.create_subscription(String, "/mission_status", self._on_status, 10)
        self.create_subscription(String, "/tts_say", self._on_say, 10)

        self._queue = queue.Queue()
        self._stop = threading.Event()
        self._backend, self._backend_name = self._init_backend(backend_param)

        self._worker = threading.Thread(target=self._speak_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"tts_node up (backend={self._backend_name}, player={self._player!r}, "
            f"speak_status={self.speak_status})"
        )
        if self._speak_intro:
            self._enqueue("AutoBee voice ready.")

    # ----- backend init -----

    def _init_backend(self, requested):
        order = [requested] if requested != "auto" else ["gtts", "pyttsx3", "print"]
        for name in order:
            backend = self._try_init(name)
            if backend is not None:
                return backend, name
        return self._init_print(), "print"

    def _try_init(self, name):
        if name == "gtts":
            return self._init_gtts()
        if name == "pyttsx3":
            return self._init_pyttsx3()
        if name == "print":
            return self._init_print()
        self.get_logger().warn(f"unknown TTS backend {name!r}")
        return None

    def _init_gtts(self):
        try:
            from gtts import gTTS
        except ImportError:
            self.get_logger().warn("gTTS not installed (pip install gtts)")
            return None
        if not self._player:
            self.get_logger().warn("no audio player available; gTTS backend disabled")
            return None
        return self._make_gtts_speaker()

    def _make_gtts_speaker(self):
        from gtts import gTTS

        def speak(text):
            with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as fh:
                path = fh.name
            try:
                gTTS(text=text, lang=self.language).save(path)
                self._play_file(path)
            finally:
                try:
                    os.unlink(path)
                except OSError:
                    pass
        return speak

    def _init_pyttsx3(self):
        try:
            import pyttsx3
        except ImportError:
            self.get_logger().warn("pyttsx3 not installed (pip install pyttsx3)")
            return None
        try:
            engine = pyttsx3.init()
        except Exception as e:
            self.get_logger().warn(f"pyttsx3 init failed: {e}")
            return None

        def speak(text):
            try:
                engine.say(text)
                engine.runAndWait()
            except Exception as e:
                self.get_logger().error(f"pyttsx3 speak failed: {e}")
        return speak

    def _init_print(self):
        def speak(text):
            self.get_logger().warn(f"[TTS no-op] {text}")
        return speak

    @staticmethod
    def _autodetect_player():
        for binary in ("mpg123", "ffplay", "mpv", "cvlc"):
            if shutil.which(binary):
                return binary
        return ""

    def _play_file(self, path):
        if not self._player:
            self.get_logger().error(
                "no audio player binary on PATH (need mpg123/ffplay/mpv); "
                "install one or set audio_player param"
            )
            return
        cmd_map = {
            "mpg123": ["mpg123", "-q", path],
            "ffplay": ["ffplay", "-nodisp", "-autoexit", "-loglevel", "error", path],
            "mpv":    ["mpv", "--no-terminal", "--really-quiet", path],
            "cvlc":   ["cvlc", "--play-and-exit", "--quiet", path],
        }
        cmd = cmd_map.get(self._player, [self._player, path])
        try:
            subprocess.run(cmd, check=False)
        except FileNotFoundError as e:
            self.get_logger().error(f"audio player {self._player!r} failed: {e}")

    # ----- subscribers -----

    def _on_report(self, msg: String):
        text = (msg.data or "").strip()
        if text:
            self._enqueue(text)

    def _on_say(self, msg: String):
        text = (msg.data or "").strip()
        if text:
            self._enqueue(text)

    def _on_status(self, msg: String):
        if not self.speak_status:
            return
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = status.get("event")
        if event == "started":
            self._enqueue("Mission started.")
        elif event == "completed":
            self._enqueue("Mission complete. Stand by for report.")
        elif event == "aborted":
            reason = status.get("reason") or "operator request"
            self._enqueue(f"Mission aborted: {reason}.")
        elif event == "failed":
            reason = status.get("reason") or "unknown failure"
            self._enqueue(f"Mission failed: {reason}.")

    # ----- worker -----

    def _enqueue(self, text):
        if len(text) > self.max_chars:
            text = text[: self.max_chars].rstrip() + "..."
        self._queue.put(text)

    def _speak_loop(self):
        while not self._stop.is_set():
            try:
                text = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            self._publish_speaking("started", text)
            try:
                self._backend(text)
            except Exception as e:
                self.get_logger().error(f"TTS backend raised: {e}")
            self._publish_speaking("finished", text)

    def _publish_speaking(self, state, text):
        msg = String()
        msg.data = json.dumps({"state": state, "text": text})
        self.speaking_pub.publish(msg)

    def stop(self):
        self._stop.set()


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
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
