"""Pure waypoint generators for survey patterns.

No ROS imports — runnable and testable as a plain script.
"""

import math
from typing import List, Tuple

Waypoint = Tuple[float, float, float]


def spiral(
    center_xy: Tuple[float, float],
    radius_m: float,
    altitude_m: float,
    arc_spacing_m: float = 4.0,
    points_per_rev: int = 16,
) -> List[Waypoint]:
    """Expanding Archimedean spiral starting at center, ending at radius_m.

    arc_spacing_m: radial distance between adjacent loops of the spiral.
    points_per_rev: how finely the curve is sampled (higher = smoother).
    Returns the list of (x, y, altitude) waypoints, then the home pose appended
    so the executor naturally returns to start.
    """
    cx, cy = center_xy
    a = arc_spacing_m / (2.0 * math.pi)
    theta_max = (radius_m / arc_spacing_m) * 2.0 * math.pi
    n_steps = max(1, int(points_per_rev * theta_max / (2.0 * math.pi)))
    step = theta_max / n_steps

    pts: List[Waypoint] = []
    for i in range(1, n_steps + 1):
        theta = i * step
        r = a * theta
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        pts.append((x, y, altitude_m))

    pts.append((cx, cy, altitude_m))
    return pts


def orbit(
    center_xy: Tuple[float, float],
    radius_m: float,
    altitude_m: float,
    points: int = 24,
) -> List[Waypoint]:
    """Single circular orbit at fixed radius. Cheaper than spiral, less coverage."""
    cx, cy = center_xy
    pts: List[Waypoint] = []
    for i in range(points):
        theta = (2.0 * math.pi) * (i / points)
        x = cx + radius_m * math.cos(theta)
        y = cy + radius_m * math.sin(theta)
        pts.append((x, y, altitude_m))
    pts.append(pts[0])
    pts.append((cx, cy, altitude_m))
    return pts


def generate(
    pattern: str,
    center_xy: Tuple[float, float],
    radius_m: float,
    altitude_m: float,
) -> List[Waypoint]:
    """Dispatch by pattern name. Matches the PRD survey_local_area schema."""
    if pattern == "spiral":
        return spiral(center_xy, radius_m, altitude_m)
    if pattern == "orbit":
        return orbit(center_xy, radius_m, altitude_m)
    raise ValueError(f"Unknown pattern: {pattern!r} (supported: spiral, orbit)")


if __name__ == "__main__":
    print("# spiral demo: center (0,0), radius 20m, altitude 12m")
    for i, (x, y, z) in enumerate(generate("spiral", (0.0, 0.0), 20.0, 12.0)):
        print(f"  {i:3d}: ({x:+7.2f}, {y:+7.2f}, {z:+6.2f})")
