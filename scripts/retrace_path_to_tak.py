#!/usr/bin/env python3
"""
retrace_path_to_tak.py
================================================================================
Ask a robot's mission_database "what's the way back?" and draw the answer on
ATAK.  A visual check for the GetRetracePath service, and a quick way to see
what the robot WOULD do on comms loss without touching the behavior tree.

Companion to db_tak_replay.py: same delivery path (CoT XML published as
std_msgs/String on /{robot_name}/send_to_tak, relayed by tak_bridge), same CoT
conventions, same deterministic-UID trick so re-running updates the drawing in
place instead of piling up duplicates.

WHAT APPEARS ON ATAK
--------------------
    - One numbered cyan spot-map point per path point: R00, R01, R02 ...
      R00 is the first point the robot would drive to; the numbers count up
      toward home.
    - A cyan line joining them (starting at the robot if its position is known).
    - The last point is labelled {ROBOT}_RETRACE_END, or HOME when the path
      ends at the stored home position.

USAGE (inside the phoenix-r2 container, tak_bridge running for that robot)
-----
    # Path all the way home from the robot's current position
    python3 retrace_path_to_tak.py warthog1

    # Only the first 30 m of the way back (what a "back out to margin" would do)
    python3 retrace_path_to_tak.py warthog1 --max_path_m 30

    # Thin the drawn path to points >= 20 m apart
    python3 retrace_path_to_tak.py warthog1 --min_spacing_m 20

    # Start from a hand-picked position instead of the GPS topic
    python3 retrace_path_to_tak.py warthog1 --from_lat 39.35264 --from_lon -76.34541

    # Skip GPS entirely: the service starts from the newest breadcrumb
    python3 retrace_path_to_tak.py warthog1 --no_gps

    # Remove the last drawing for this robot from ATAK
    python3 retrace_path_to_tak.py warthog1 --clear

ARGUMENTS
---------
    robot_name            Robot namespace, e.g. warthog1 (required)
    --gps_topic TOPIC     NavSatFix topic for the robot's position.
                          Default: /{robot_name}/sensors/ublox/fix
    --gps_timeout SEC     How long to wait for one fix (default 5.0).  If none
                          arrives the script warns and falls back to --no_gps.
    --from_lat / --from_lon
                          Use this position instead of the GPS topic.
    --no_gps              Don't read GPS; service starts from the newest crumb.
    --max_path_m FLOAT    0 = whole trail + home (default).  >0 = stop after
                          that many metres of path; home is NOT appended.
    --min_spacing_m FLOAT 0 = every crumb (default).  >0 = thin the path.
    --service_timeout SEC Wait for the service to appear / answer (default 10).
    --retries INT         Sends per CoT message (default 3, as db_tak_replay
                          does 5).  30 points x 3 sends x 0.1 s ~ 9 s.
    --clear               Delete this robot's previous retrace drawing and exit.

HOW RE-RUNS ARE KEPT TIDY
-------------------------
    Every point and the line get a UID derived from (robot_name, index), so
    a second run overwrites the first on ATAK.  Points left over from a longer
    previous path are deleted explicitly; what was sent last time is remembered
    in ~/.cache/retrace_path_to_tak/{robot_name}.json.

WHAT THIS DOES NOT DO
---------------------
    It never writes to the database and never talks to Maestro.  It only reads
    the path the service would hand to the behavior tree, and draws it.
================================================================================
"""

import argparse
import json
import math
import sys
import time
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

try:
    from mission_database.srv import GetRetracePath
except ImportError:
    sys.exit(
        'ERROR: mission_database/srv/GetRetracePath not found.\n'
        '       Build mission_database with the new service and '
        '`source install/setup.bash` first.'
    )


# =============================================================================
# CONSTANTS
# =============================================================================

# Cyan (0xFF00FFFF).  Distinct from db_tak_replay's white/red breadcrumbs and
# the yellow/green/red waypoint circles, so a retrace drawing is obvious.
_COLOR_ARGB   = '-16711681'
_STALE_DAYS   = 365          # keep the drawing on ATAK until cleared
_STATE_DIR    = Path.home() / '.cache' / 'retrace_path_to_tak'


def _now_strings() -> tuple[str, str]:
    now   = datetime.now(timezone.utc)
    stale = now + timedelta(days=_STALE_DAYS)
    return now.strftime('%Y-%m-%dT%H:%M:%SZ'), stale.strftime('%Y-%m-%dT%H:%M:%SZ')


# Deterministic UIDs: same robot + index -> same UID -> ATAK updates in place.
def _point_uid(robot_name: str, index: int) -> str:
    return str(uuid.uuid5(uuid.NAMESPACE_URL,
                          f'mission_database/{robot_name}/retrace/point/{index}'))


def _line_uid(robot_name: str) -> str:
    return str(uuid.uuid5(uuid.NAMESPACE_URL,
                          f'mission_database/{robot_name}/retrace/line'))


# =============================================================================
# COT BUILDERS
#
# Same shapes as db_tak_replay.py, so anything that renders there renders here.
# =============================================================================

def build_point_cot(robot_name: str, index: int, lat: float, lon: float,
                    callsign: str) -> str:
    """
    A CoT b-m-p-s-m spot-map point, like db_tak_replay's breadcrumb but with a
    <contact callsign> so the order is readable on the map.
    """
    time_str, stale_str = _now_strings()
    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        f'<event version="2.0"'
        f' uid="{_point_uid(robot_name, index)}"'
        f' type="b-m-p-s-m"'
        f' how="h-g-i-g-o"'
        f' time="{time_str}"'
        f' start="{time_str}"'
        f' stale="{stale_str}"'
        f' access="Undefined">'
        f'<point lat="{lat:.7f}" lon="{lon:.7f}" hae="0.0" ce="10.0" le="10.0"/>'
        f'<detail>'
        f'<contact callsign="{callsign}"/>'
        f'<precisionlocation geopointsrc="GPS" altsrc="GPS"/>'
        f'<status readiness="true"/>'
        f'<archive/>'
        f'<creator uid="{robot_name}" callsign="{robot_name}"'
        f' time="{time_str}" type="a-f-G-E-V"/>'
        f'<usericon iconsetpath="COT_MAPPING_SPOTMAP/b-m-p-s-m/{_COLOR_ARGB}"/>'
        f'<color argb="{_COLOR_ARGB}"/>'
        f'<link uid="{robot_name}" production_time="{time_str}"'
        f' type="a-f-G-E-V" parent_callsign="{robot_name}" relation="p-p"/>'
        f'<remarks/>'
        f'</detail>'
        f'</event>'
    )


def build_line_cot(robot_name: str, vertices: list[tuple[float, float]]) -> str:
    """
    A CoT u-d-f (drawing line) through `vertices`, first to last.

    Kept OPEN on purpose: no <fillColor> element.  ATAK treats a u-d-f with a
    fill as a closed polygon, which would draw a bogus leg from the last point
    back to the first.  If your ATAK build still closes it, delete the line
    and rely on the numbered points -- they carry the same information.
    """
    time_str, stale_str = _now_strings()
    lat0, lon0 = vertices[0]
    links = ''.join(f'<link point="{lat:.7f},{lon:.7f}"/>' for lat, lon in vertices)
    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        f'<event version="2.0"'
        f' uid="{_line_uid(robot_name)}"'
        f' type="u-d-f"'
        f' how="h-e"'
        f' time="{time_str}"'
        f' start="{time_str}"'
        f' stale="{stale_str}"'
        f' access="Undefined">'
        f'<point lat="{lat0:.7f}" lon="{lon0:.7f}" hae="0.0"'
        f' ce="9999999.0" le="9999999.0"/>'
        f'<detail>'
        f'<contact callsign="{robot_name.upper()}_RETRACE"/>'
        f'{links}'
        f'<strokeColor value="{_COLOR_ARGB}"/>'
        f'<strokeWeight value="3.0"/>'
        f'<strokeStyle value="solid"/>'
        f'<labels_on value="false"/>'
        f'<archive/>'
        f'<remarks/>'
        f'</detail>'
        f'</event>'
    )


def build_delete_cot(target_uid: str, lat: float, lon: float) -> str:
    """Identical to db_tak_replay.build_delete_cot_message."""
    now       = datetime.now(timezone.utc)
    stale     = now + timedelta(minutes=1)
    time_str  = now.strftime('%Y-%m-%dT%H:%M:%SZ')
    stale_str = stale.strftime('%Y-%m-%dT%H:%M:%SZ')
    cmd_uid   = f'delete-cmd-{str(uuid.uuid4())[:8]}'
    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        f'<event version="2.0"'
        f' uid="{cmd_uid}"'
        f' type="t-x-d-d"'
        f' how="m-g"'
        f' time="{time_str}"'
        f' start="{time_str}"'
        f' stale="{stale_str}">'
        f'<point lat="{lat:.7f}" lon="{lon:.7f}" hae="0.0"'
        f' ce="9999999.0" le="9999999.0"/>'
        f'<detail>'
        f'<link uid="{target_uid}" relation="none" type="none"/>'
        f'<__forcedelete/>'
        f'</detail>'
        f'</event>'
    )


# =============================================================================
# ROS NODE
# =============================================================================

class RetraceNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f'retrace_path_to_tak_{robot_name}')
        self._topic = f'/{robot_name}/send_to_tak'
        self._pub   = self.create_publisher(String, self._topic, qos_profile=10)
        self._client = self.create_client(
            GetRetracePath, f'/{robot_name}/mission_database/retrace_path')
        self.get_logger().info(f'[retrace_path_to_tak] Publishing CoT to: {self._topic}')

    def publish(self, xml: str) -> None:
        msg      = String()
        msg.data = xml
        self._pub.publish(msg)

    def publish_with_retry(self, xml: str, retries: int, delay: float = 0.1) -> None:
        """Same redundancy trick as db_tak_replay: send N times, spin between."""
        for _ in range(retries):
            self.publish(xml)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(delay)

    def wait_for_one_fix(self, topic: str, timeout: float) -> tuple[float, float] | None:
        """Block until one NavSatFix with finite lat/lon arrives, or time out."""
        got: dict = {}

        def _cb(msg: NavSatFix) -> None:
            if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
                got['fix'] = (msg.latitude, msg.longitude)

        sub = self.create_subscription(NavSatFix, topic, _cb, 10)
        deadline = time.monotonic() + timeout
        while 'fix' not in got and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return got.get('fix')

    def call_retrace(self, from_lat: float, from_lon: float, max_path_m: float,
                     min_spacing_m: float, timeout: float):
        """Call the service; returns the response or None."""
        if not self._client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f'Service {self._client.srv_name} not available after {timeout:.0f} s. '
                'Is the stack (mission_database_node) running for this robot?')
            return None
        req = GetRetracePath.Request()
        req.from_lat      = from_lat
        req.from_lon      = from_lon
        req.max_path_m    = max_path_m
        req.min_spacing_m = min_spacing_m
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error(f'Service call timed out after {timeout:.0f} s.')
            return None
        return future.result()


# =============================================================================
# STATE FILE (what was drawn last time, so leftovers can be removed)
# =============================================================================

def _state_path(robot_name: str) -> Path:
    return _STATE_DIR / f'{robot_name}.json'


def load_state(robot_name: str) -> dict:
    try:
        return json.loads(_state_path(robot_name).read_text())
    except (OSError, ValueError):
        return {'points': [], 'line': None}


def save_state(robot_name: str, state: dict) -> None:
    _STATE_DIR.mkdir(parents=True, exist_ok=True)
    _state_path(robot_name).write_text(json.dumps(state))


def clear_previous(node: RetraceNode, robot_name: str, state: dict,
                   keep_first_n: int, retries: int) -> int:
    """
    Delete points with index >= keep_first_n (they are not being redrawn) and
    the old line if there is no new one.  Returns how many deletes were sent.
    """
    sent = 0
    for index, (lat, lon) in enumerate(state.get('points', [])):
        if index >= keep_first_n:
            node.publish_with_retry(build_delete_cot(_point_uid(robot_name, index), lat, lon),
                                    retries)
            sent += 1
    if state.get('line') and keep_first_n == 0:
        lat, lon = state['line']
        node.publish_with_retry(build_delete_cot(_line_uid(robot_name), lat, lon), retries)
        sent += 1
    return sent


# =============================================================================
# TERMINAL SUMMARY
# =============================================================================

def _haversine_m(lat1, lon1, lat2, lon2) -> float:
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi   = math.radians(lat2 - lat1)
    dlam   = math.radians(lon2 - lon1)
    h = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


def print_summary(resp, start: tuple[float, float] | None) -> None:
    print(f'\nService: {resp.message}')
    print(f'  breadcrumbs read : {resp.crumbs_total}   erased as out-and-back: {resp.crumbs_erased}')
    print(f'  path length      : {resp.path_length_m:.1f} m   ends at home: {resp.ends_at_home}')
    print(f'  start            : '
          + (f'robot at {start[0]:.6f}, {start[1]:.6f}' if start else 'newest breadcrumb'))
    if not resp.lats:
        return
    print(f'\n  {"#":>3}  {"lat":>11}  {"lon":>12}  {"leg m":>6}  {"total m":>8}')
    prev, total = start, 0.0
    for i, (lat, lon) in enumerate(zip(resp.lats, resp.lons)):
        leg = _haversine_m(prev[0], prev[1], lat, lon) if prev else 0.0
        total += leg
        tag = '  <- home' if (resp.ends_at_home and i == len(resp.lats) - 1) else ''
        print(f'  {i:>3}  {lat:>11.6f}  {lon:>12.6f}  {leg:>6.1f}  {total:>8.1f}{tag}')
        prev = (lat, lon)


# =============================================================================
# MAIN
# =============================================================================

def main() -> None:
    ap = argparse.ArgumentParser(
        description='Draw the way back (GetRetracePath) on ATAK for one robot.')
    ap.add_argument('robot_name')
    ap.add_argument('--gps_topic', default=None,
                    help='NavSatFix topic (default /{robot_name}/sensors/ublox/fix)')
    ap.add_argument('--gps_timeout', type=float, default=5.0)
    ap.add_argument('--from_lat', type=float, default=None)
    ap.add_argument('--from_lon', type=float, default=None)
    ap.add_argument('--no_gps', action='store_true')
    ap.add_argument('--max_path_m', type=float, default=0.0)
    ap.add_argument('--min_spacing_m', type=float, default=0.0)
    ap.add_argument('--service_timeout', type=float, default=10.0)
    ap.add_argument('--retries', type=int, default=3)
    ap.add_argument('--clear', action='store_true',
                    help='delete the previous drawing for this robot and exit')
    args = ap.parse_args()

    robot = args.robot_name
    if (args.from_lat is None) != (args.from_lon is None):
        ap.error('--from_lat and --from_lon must be given together')

    rclpy.init()
    node = RetraceNode(robot)
    state = load_state(robot)

    try:
        # ── CLEAR ONLY ──────────────────────────────────────────────────────
        if args.clear:
            n = clear_previous(node, robot, state, keep_first_n=0, retries=args.retries)
            save_state(robot, {'points': [], 'line': None})
            print(f'Sent {n} delete(s) for {robot}.')
            return

        # ── WHERE IS THE ROBOT? ─────────────────────────────────────────────
        start: tuple[float, float] | None = None
        if args.from_lat is not None:
            start = (args.from_lat, args.from_lon)
        elif not args.no_gps:
            topic = args.gps_topic or f'/{robot}/sensors/ublox/fix'
            print(f'Waiting up to {args.gps_timeout:.0f} s for a fix on {topic} ...')
            start = node.wait_for_one_fix(topic, args.gps_timeout)
            if start is None:
                print('  no fix received -- falling back to the newest breadcrumb '
                      '(use --gps_topic if the topic name is different)')

        # ── ASK THE DATABASE ────────────────────────────────────────────────
        resp = node.call_retrace(
            from_lat=start[0] if start else 0.0,
            from_lon=start[1] if start else 0.0,
            max_path_m=args.max_path_m,
            min_spacing_m=args.min_spacing_m,
            timeout=args.service_timeout,
        )
        if resp is None:
            return
        if not resp.success:
            print(f'Service reported failure: {resp.message}')
            return
        print_summary(resp, start)

        points = list(zip(resp.lats, resp.lons))
        if not points:
            n = clear_previous(node, robot, state, keep_first_n=0, retries=args.retries)
            save_state(robot, {'points': [], 'line': None})
            print(f'\nNothing to draw.  Removed {n} leftover symbol(s) from the last run.')
            return

        # ── DRAW ────────────────────────────────────────────────────────────
        print(f'\nDrawing {len(points)} point(s) + line on ATAK ...')
        last = len(points) - 1
        for i, (lat, lon) in enumerate(points):
            if i == last:
                callsign = 'HOME' if resp.ends_at_home else f'{robot.upper()}_RETRACE_END'
            else:
                callsign = f'R{i:02d}'
            node.publish_with_retry(build_point_cot(robot, i, lat, lon, callsign), args.retries)

        vertices = ([start] if start else []) + points
        line_anchor = None
        if len(vertices) >= 2:
            node.publish_with_retry(build_line_cot(robot, vertices), args.retries)
            line_anchor = list(vertices[0])

        # ── TIDY UP LEFTOVERS FROM A LONGER PREVIOUS RUN ────────────────────
        n = clear_previous(node, robot, state, keep_first_n=len(points), retries=args.retries)
        if state.get('line') and line_anchor is None:
            lat, lon = state['line']
            node.publish_with_retry(build_delete_cot(_line_uid(robot), lat, lon), args.retries)
            n += 1
        save_state(robot, {'points': [list(p) for p in points], 'line': line_anchor})

        rclpy.spin_once(node, timeout_sec=0.1)
        print(f'Done.  {len(points)} point(s) drawn'
              + (f', {n} leftover(s) removed' if n else '')
              + f'.  Re-run to refresh, or --clear to remove.')

    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
