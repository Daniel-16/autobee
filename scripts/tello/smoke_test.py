#!/usr/bin/env python3
"""Offline Tello smoke test. No internet required; only the Tello AP.

    python3 scripts/tello/smoke_test.py                    # state only, no flight
    python3 scripts/tello/smoke_test.py --hover            # state + 5s hover + land
    python3 scripts/tello/smoke_test.py --hover --video    # hover with live camera window
    python3 scripts/tello/smoke_test.py --video            # camera preview only, no flight
"""

import argparse
import signal
import sys
import time

from djitellopy import Tello

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

MIN_BATTERY_FOR_FLIGHT = 30
MAX_TEMP_C = 80
WINDOW_NAME = "Tello camera"


def banner(title: str) -> None:
    print(f"\n=== {title} ===")


def safe_call(label: str, fn):
    try:
        result = fn()
        if isinstance(result, str) and result.startswith("unknown command"):
            return label, "n/a (Tello EDU only)"
        return label, result
    except Exception as exc:
        return label, f"ERR: {exc}"


def dump_state(t: Tello) -> None:
    rows = [
        safe_call("battery (%)",          t.get_battery),
        safe_call("flight time (s)",      t.get_flight_time),
        safe_call("temperature avg (C)",  t.get_temperature),
        safe_call("temperature low (C)",  t.get_lowest_temperature),
        safe_call("temperature high (C)", t.get_highest_temperature),
        safe_call("height (cm)",          t.get_height),
        safe_call("ToF distance (cm)",    t.get_distance_tof),
        safe_call("barometer (cm)",       t.get_barometer),
        safe_call("pitch (deg)",          t.get_pitch),
        safe_call("roll (deg)",           t.get_roll),
        safe_call("yaw (deg)",            t.get_yaw),
        safe_call("vx (cm/s)",            t.get_speed_x),
        safe_call("vy (cm/s)",            t.get_speed_y),
        safe_call("vz (cm/s)",            t.get_speed_z),
        safe_call("ax (0.001g)",          t.get_acceleration_x),
        safe_call("ay (0.001g)",          t.get_acceleration_y),
        safe_call("az (0.001g)",          t.get_acceleration_z),
        safe_call("SDK version",          t.query_sdk_version),
        safe_call("serial number",        t.query_serial_number),
        safe_call("Wi-Fi SNR",            t.query_wifi_signal_noise_ratio),
    ]
    width = max(len(r[0]) for r in rows)
    for label, value in rows:
        print(f"  {label.ljust(width)} : {value}")


def preflight(t: Tello) -> None:
    banner("PRE-FLIGHT CHECKS")
    battery = t.get_battery()
    temp = t.get_temperature()
    print(f"  battery     : {battery}%   (min for flight: {MIN_BATTERY_FOR_FLIGHT}%)")
    print(f"  temperature : {temp}C      (max safe: {MAX_TEMP_C}C)")
    if battery < MIN_BATTERY_FOR_FLIGHT:
        sys.exit(f"REFUSING TO FLY: battery {battery}% < {MIN_BATTERY_FOR_FLIGHT}%. Charge and retry.")
    if temp > MAX_TEMP_C:
        sys.exit(f"REFUSING TO FLY: temperature {temp}C > {MAX_TEMP_C}C. Cool down and retry.")
    print("  -> OK")


def start_video(t: Tello):
    """Open Tello stream + cv2 window. Returns frame_read or None on failure."""
    if not _HAS_CV2:
        print("  WARN: cv2 not importable — skipping video.")
        return None
    # opencv-python-headless lacks GUI symbols; namedWindow proves them out early.
    try:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    except cv2.error as exc:
        print(f"  WARN: cv2 GUI not available ({exc}).")
        print("        Fix: pip install --force-reinstall opencv-python")
        return None
    t.streamon()
    fr = t.get_frame_read()
    time.sleep(1.0)  # let the H.264 decoder buffer fill
    return fr


def show_frame(fr) -> None:
    if fr is None:
        return
    f = fr.frame
    if f is not None:
        cv2.imshow(WINDOW_NAME, f)
        cv2.waitKey(1)


def stop_video(t: Tello, fr) -> None:
    if fr is None:
        return
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    try:
        t.streamoff()
    except Exception:
        pass


def hover(t: Tello, secs: float, fr=None) -> None:
    banner(f"HOVER {secs}s")
    print("  takeoff ...")
    t.takeoff()
    print(f"  holding zero RC for {secs}s ...")
    deadline = time.time() + secs
    while time.time() < deadline:
        t.send_rc_control(0, 0, 0, 0)
        show_frame(fr)
        time.sleep(0.1)
    print("  landing ...")
    t.land()
    print("  -> landed")


def video_preview(fr, secs: float) -> None:
    if fr is None:
        return
    banner(f"VIDEO PREVIEW {secs}s (no flight)")
    print("  press Ctrl+C to exit early")
    deadline = time.time() + secs
    while time.time() < deadline:
        show_frame(fr)
        time.sleep(0.05)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hover", action="store_true", help="Take off, hover, land. Default: state only.")
    ap.add_argument("--hover-secs", type=float, default=5.0, help="Hover duration in seconds (default 5).")
    ap.add_argument("--video", action="store_true", help="Open a window showing the Tello camera feed.")
    ap.add_argument("--video-secs", type=float, default=10.0,
                    help="With --video and no --hover, how long to show the preview (default 10).")
    args = ap.parse_args()

    banner("CONNECTING")
    t = Tello()
    t.connect()
    print("  -> connected to 192.168.10.1")

    fr = None
    def _on_sigint(signum, frame):
        print("\n  SIGINT -> land()")
        try:
            t.land()
        except Exception:
            try:
                t.emergency()
            except Exception:
                pass
        stop_video(t, fr)
        sys.exit(130)
    signal.signal(signal.SIGINT, _on_sigint)

    banner("STATE")
    dump_state(t)

    if args.video:
        fr = start_video(t)

    try:
        if args.hover:
            preflight(t)
            hover(t, args.hover_secs, fr=fr)
            banner("POST-FLIGHT STATE")
            dump_state(t)
        elif args.video:
            video_preview(fr, args.video_secs)
    finally:
        stop_video(t, fr)

    print("\nDONE.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
