"""AutoBee dashboard bridge — the ROS <-> web API behind the Next.js front-end.

A single ROS 2 node that exposes the mission topic graph over a small HTTP API
(stdlib only, no extra Python deps). The Next.js dashboard in `dashboard/`
consumes this bridge. Endpoints (all CORS-enabled so the Next.js dev server can
reach them cross-origin):

  GET  /state        -> current snapshot (JSON)
  GET  /events       -> Server-Sent Events stream of snapshots (live updates)
  POST /intent       -> {"text": "..."} published to /user_intent
  GET  /camera.mjpg  -> annotated camera as MJPEG (only if OpenCV/cv_bridge load)
  GET  /             -> tiny human-readable status page (sanity check)

It subscribes to /mission_status, /mission_command, /mission_report,
/crowd_analytics, /anomaly_alert, /perception/person_count and /drone/odom, and
publishes to /user_intent. The camera is best-effort: if cv_bridge/OpenCV don't
import, the bridge still runs and just reports camera_available=false.

Run standalone (against an already-running stack):

    ros2 run autobee_ui web_ui_node

or it comes up automatically with `demo.launch.py`. On a Mac, start the
container with `-p 8080:8080` so the host browser / Next.js app can reach it.
Then in `dashboard/` run `npm install && npm run dev` and open localhost:3000.
"""

import json
import math
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry

# Camera is best-effort: if the image stack isn't importable, we degrade
# gracefully to a dashboard with no video panel rather than failing to launch.
try:
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    _CAMERA_AVAILABLE = True
except Exception:  # pragma: no cover - depends on container build
    _CAMERA_AVAILABLE = False

MAX_ANOMALIES = 8          # how many recent alerts to keep on screen
MAX_EVENTS = 12            # rolling mission-event log length


class UIState:
    """Thread-safe snapshot of everything the dashboard shows.

    ROS callbacks (spin thread) write here; HTTP handler threads read here and
    fan snapshots out to connected SSE clients.
    """

    def __init__(self, camera_available):
        self._lock = threading.Lock()
        self._clients = set()          # set[queue.Queue] — one per SSE client
        self._jpeg = None              # latest annotated frame as JPEG bytes
        self.data = {
            "camera_available": camera_available,
            "mission": {
                "event": "idle", "phase": "—", "intent": "", "landmark": None,
                "reason": "", "wall_time": 0.0,
            },
            "plan": None,
            "report": "",
            "analytics": None,
            "anomalies": [],          # most-recent first
            "events": [],             # rolling mission status log, most-recent first
            "telemetry": {
                "x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0,
                "speed": 0.0, "have_odom": False,
            },
            "person_count": 0,
            "last_intent": "",
        }

    # ---- state mutation (called from ROS thread) ----
    def set(self, key, value):
        with self._lock:
            self.data[key] = value

    def update_mission(self, payload):
        with self._lock:
            m = self.data["mission"]
            for k in ("event", "phase", "intent", "landmark", "reason", "wall_time"):
                if k in payload and payload[k] is not None:
                    m[k] = payload[k]
            # roll a human-readable event line
            label = payload.get("event", "")
            phase = payload.get("phase", "")
            line = {
                "t": time.time(),
                "text": f"{label} · {phase}".strip(" ·"),
                "reason": payload.get("reason", ""),
            }
            evs = self.data["events"]
            evs.insert(0, line)
            del evs[MAX_EVENTS:]

    def add_anomaly(self, payload):
        with self._lock:
            self.data["anomalies"].insert(0, payload)
            del self.data["anomalies"][MAX_ANOMALIES:]

    def update_telemetry(self, x, y, z, yaw_deg, speed):
        with self._lock:
            self.data["telemetry"] = {
                "x": round(x, 2), "y": round(y, 2), "z": round(z, 2),
                "yaw_deg": round(yaw_deg, 1), "speed": round(speed, 2),
                "have_odom": True,
            }

    # ---- JPEG frame for the MJPEG stream ----
    def set_jpeg(self, jpeg_bytes):
        with self._lock:
            self._jpeg = jpeg_bytes

    def get_jpeg(self):
        with self._lock:
            return self._jpeg

    # ---- SSE client bookkeeping ----
    def add_client(self, q):
        with self._lock:
            self._clients.add(q)

    def remove_client(self, q):
        with self._lock:
            self._clients.discard(q)

    def snapshot_json(self):
        with self._lock:
            return json.dumps(self.data)

    def broadcast(self):
        msg = self.snapshot_json()
        with self._lock:
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass  # slow client — drop this frame, it'll get the next one


def _make_handler(state, publish_intent, publish_abort, logger):
    """Build the HTTP request handler bound to our state + publishers."""

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):  # silence default stderr spam
            pass

        # -- helpers --
        def _send(self, code, ctype, body, extra_headers=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            for k, v in (extra_headers or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, "application/json",
                       json.dumps(obj).encode("utf-8"))

        # -- routes --
        def do_OPTIONS(self):
            # CORS preflight for the POST /intent call from the Next.js app.
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                cam = "on" if state.data.get("camera_available") else "off"
                page = (
                    "<!doctype html><meta charset=utf-8>"
                    "<title>AutoBee bridge</title>"
                    "<body style='font-family:sans-serif;background:#0d1117;"
                    "color:#e6edf3;padding:40px'>"
                    "<h1>\U0001F41D AutoBee dashboard bridge</h1>"
                    "<p>This is the ROS↔web API. The dashboard UI is the "
                    "Next.js app in <code>dashboard/</code>.</p>"
                    "<p>Endpoints: <code>/state</code> · <code>/events</code> "
                    "(SSE) · <code>POST /intent</code> · "
                    "<code>/camera.mjpg</code></p>"
                    f"<p>Camera: <b>{cam}</b></p></body>"
                )
                self._send(200, "text/html; charset=utf-8", page.encode("utf-8"))
            elif self.path == "/state":
                self._send(200, "application/json",
                           state.snapshot_json().encode("utf-8"))
            elif self.path == "/events":
                self._serve_sse()
            elif self.path == "/camera.mjpg":
                self._serve_mjpeg()
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self):
            if self.path == "/intent":
                length = int(self.headers.get("Content-Length", 0) or 0)
                raw = self.rfile.read(length) if length else b""
                try:
                    text = json.loads(raw or b"{}").get("text", "").strip()
                except Exception:
                    text = ""
                if not text:
                    self._json(400, {"ok": False, "error": "empty command"})
                    return
                publish_intent(text)
                state.set("last_intent", text)
                state.broadcast()
                self._json(200, {"ok": True, "text": text})
            elif self.path == "/abort":
                # Emergency: tell the drone to land now (kill switch).
                publish_abort()
                self._json(200, {"ok": True})
            else:
                self._json(404, {"error": "not found"})

        # -- streaming endpoints --
        def _serve_sse(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            q = queue.Queue(maxsize=16)
            state.add_client(q)
            try:
                self._sse_write(state.snapshot_json())  # prime on connect
                while True:
                    try:
                        msg = q.get(timeout=15)
                        self._sse_write(msg)
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")  # keep-alive comment
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ValueError):
                pass
            finally:
                state.remove_client(q)

        def _sse_write(self, data):
            self.wfile.write(b"data: " + data.encode("utf-8") + b"\n\n")
            self.wfile.flush()

        def _serve_mjpeg(self):
            if not state.data.get("camera_available"):
                self._json(404, {"error": "camera not available"})
                return
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    frame = state.get_jpeg()
                    if frame is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            ("Content-Length: %d\r\n\r\n" % len(frame)).encode())
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    time.sleep(0.1)  # ~10 fps cap
            except (BrokenPipeError, ConnectionResetError, ValueError):
                pass

    return Handler


class WebUINode(Node):
    def __init__(self):
        super().__init__("web_ui_node")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("enable_camera", True)
        # Live drone camera by default (full ~10 FPS). The YOLO-annotated topic
        # (/perception/image_annotated) is only produced at inference rate, which
        # drops to <1 FPS under load and looks frozen — set this param to it only
        # if you specifically want the detection overlay over a live feed.
        self.declare_parameter("camera_topic", "/drone/camera/image_raw")
        self.declare_parameter("broadcast_hz", 4.0)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        enable_camera = bool(self.get_parameter("enable_camera").value)
        camera_topic = self.get_parameter("camera_topic").value

        self._camera_on = _CAMERA_AVAILABLE and enable_camera
        self.state = UIState(camera_available=self._camera_on)

        # ---- subscriptions: the whole topic graph the operator cares about ----
        self.create_subscription(String, "/mission_status", self._on_status, 10)
        self.create_subscription(String, "/mission_command", self._on_command, 10)
        self.create_subscription(String, "/mission_report", self._on_report, 10)
        self.create_subscription(String, "/crowd_analytics", self._on_analytics, 10)
        self.create_subscription(String, "/anomaly_alert", self._on_anomaly, 10)
        self.create_subscription(Int32, "/perception/person_count",
                                 self._on_person_count, 10)
        self.create_subscription(Odometry, "/drone/odom", self._on_odom, 10)

        # ---- publishers: command + emergency abort ----
        self.intent_pub = self.create_publisher(String, "/user_intent", 10)
        self.abort_pub = self.create_publisher(String, "/mission_abort", 10)

        # ---- camera (best effort) ----
        if self._camera_on:
            self._bridge = CvBridge()
            self.create_subscription(Image, camera_topic, self._on_image, 10)
            self.get_logger().info(f"camera feed wired to {camera_topic}")
        elif enable_camera and not _CAMERA_AVAILABLE:
            self.get_logger().warn(
                "camera requested but cv_bridge/OpenCV unavailable — "
                "dashboard will run without the video panel.")

        # ---- HTTP server in a background thread ----
        handler = _make_handler(self.state, self._publish_intent,
                                self._publish_abort, self.get_logger())
        self._httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self._httpd.daemon_threads = True
        self._server_thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True)
        self._server_thread.start()

        # ---- periodic broadcast (covers smooth telemetry updates) ----
        hz = float(self.get_parameter("broadcast_hz").value)
        self.create_timer(1.0 / max(0.5, hz), self.state.broadcast)

        shown_host = "localhost" if self.host in ("0.0.0.0", "") else self.host
        self.get_logger().info(
            f"AutoBee dashboard live at http://{shown_host}:{self.port}  "
            f"(camera: {'on' if self._camera_on else 'off'})")

    # ---------- ROS callbacks ----------
    def _on_status(self, msg):
        try:
            self.state.update_mission(json.loads(msg.data))
        except Exception:
            return
        self.state.broadcast()

    def _on_command(self, msg):
        try:
            self.state.set("plan", json.loads(msg.data))
        except Exception:
            self.state.set("plan", {"raw": msg.data})
        self.state.broadcast()

    def _on_report(self, msg):
        self.state.set("report", msg.data)
        self.state.broadcast()

    def _on_analytics(self, msg):
        try:
            self.state.set("analytics", json.loads(msg.data))
        except Exception:
            return
        # analytics broadcast rides the periodic timer (it's ~1 Hz already)

    def _on_anomaly(self, msg):
        try:
            self.state.add_anomaly(json.loads(msg.data))
        except Exception:
            return
        self.state.broadcast()

    def _on_person_count(self, msg):
        self.state.set("person_count", int(msg.data))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        # yaw from quaternion (REP-103 z-up)
        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw_deg = math.degrees(math.atan2(siny, cosy))
        v = msg.twist.twist.linear
        speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        self.state.update_telemetry(p.x, p.y, p.z, yaw_deg, speed)

    def _on_image(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            ok, buf = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                self.state.set_jpeg(buf.tobytes())
        except Exception:
            pass

    # ---------- command publish (called from HTTP thread) ----------
    def _publish_intent(self, text):
        self.intent_pub.publish(String(data=text))
        self.get_logger().info(f"dashboard → /user_intent: {text!r}")

    def _publish_abort(self):
        self.abort_pub.publish(String(data="dashboard emergency land"))
        self.get_logger().warn("dashboard → /mission_abort (emergency land)")

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
