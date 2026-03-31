#!/usr/bin/env python3
"""
db_tak_replay.py
================================================================================
Replays a mission_database SQLite file to TAK by publishing CoT spot-map
messages on /{robot_name}/send_to_tak (std_msgs/String).

Each breadcrumb in the database is sent as a CoT b-m-p-s-m (spot-map point)
message.  In timed mode the script honours the original timing of the mission
-- if breadcrumb 1 and breadcrumb 2 were recorded 5 seconds apart, the script
waits 5 seconds (adjusted for speed multiplier) before sending the second
message.  This lets you watch the robot's path animate on ATAK at a
controllable pace.

TERMINAL CONTROLS (timed mode only)
-------------------------------------
    Space / p    Pause / resume
    q            Quit cleanly

USAGE
-----
    # Real-time replay (robot name inferred from filename)
    python3 db_tak_replay.py \
        /phoenix/src/utils/mission_database/database/warthog1_2024-03-15_14-23-07.db

    # Explicit robot name
    python3 db_tak_replay.py warthog1_2024-03-15_14-23-07.db --robot_name warthog1

    # 4x speed
    python3 db_tak_replay.py warthog1_2024-03-15_14-23-07.db --speed 4.0

    # Instant -- send all breadcrumbs as fast as possible (useful for a full
    # trail overview; all points appear on ATAK nearly simultaneously)
    python3 db_tak_replay.py warthog1_2024-03-15_14-23-07.db --instant

    # Override the CoT track speed field (default 1.4 m/s)
    python3 db_tak_replay.py warthog1_2024-03-15_14-23-07.db --track_speed 2.0

ARGUMENTS
---------
    db_path             Path to the .db file to replay (required)
    --robot_name NAME   Robot namespace, e.g. warthog1.
                        Inferred from the filename if not specified:
                        'warthog1_2024-03-15_14-23-07.db' -> 'warthog1'
    --speed FLOAT       Playback speed multiplier (default: 1.0).
                        2.0 = twice as fast, 0.5 = half speed.
    --instant           Send all breadcrumbs immediately without timing delays.
                        Ignores --speed.  Pause/resume controls not available
                        in instant mode.
    --track_speed FLOAT CoT <track speed="..."> field in m/s (default: 1.4).

COT MESSAGE FORMAT
------------------
    type  : b-m-p-s-m   (spot-map point, leaves a persistent trail on ATAK)
    uid   : unique UUID4 per breadcrumb
    time  : original breadcrumb timestamp from the database
    lat   : breadcrumb latitude
    lon   : breadcrumb longitude
    course: heading from the database -- <track> element omitted if not recorded
    speed : --track_speed argument (default 1.4 m/s)
    stale : 1 year after the breadcrumb timestamp (keeps points visible)
================================================================================
"""

import argparse
import select
import sqlite3
import sys
import termios
import threading
import time
import tty
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# =============================================================================
# KEYBOARD CONTROLLER
#
# Runs a daemon thread that reads single keypresses from stdin in raw mode.
# The main replay loop checks .is_paused() and .is_quit_requested() to
# respond without blocking.
#
# Raw mode is used so keypresses take effect immediately without needing Enter.
# The original terminal settings are always restored on exit -- even on crash.
#
# Falls back gracefully when stdin is not a TTY (e.g. piped input or CI).
# =============================================================================


class KeyboardController:

    PAUSE_KEYS = {" ", "p", "P"}
    QUIT_KEYS = {
        "q",
        "Q",
    }  # Ctrl+C is SIGINT via setcbreak, caught by KeyboardInterrupt

    def __init__(self):
        self._paused = threading.Event()
        self._quit = threading.Event()
        self._tty_active = False
        self._thread = threading.Thread(
            target=self._read_keys, daemon=True, name="kb_controller"
        )

    def start(self) -> bool:
        """
        Start the keyboard listener thread.
        Returns True if raw terminal mode was successfully activated.
        Returns False if stdin is not a TTY (controls silently unavailable).
        """
        if not sys.stdin.isatty():
            return False
        self._tty_active = True
        self._thread.start()
        return True

    def is_paused(self) -> bool:
        return self._paused.is_set()

    def is_quit_requested(self) -> bool:
        return self._quit.is_set()

    def request_quit(self) -> None:
        """Called by the main thread (e.g. on KeyboardInterrupt) to signal stop."""
        self._quit.set()

    def stop(self) -> None:
        """
        Signal the keyboard thread to exit and wait for it to join.
        This restores the terminal to normal mode before any subsequent
        input() calls -- important for the delete confirmation prompt.
        """
        self._quit.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def wait_while_paused(self) -> bool:
        """
        Block until unpaused or quit is requested.
        Returns True if resumed normally, False if quit was requested.
        """
        while self._paused.is_set() and not self._quit.is_set():
            time.sleep(0.05)
        return not self._quit.is_set()

    def _read_keys(self) -> None:
        """Background thread: reads stdin one character at a time in raw mode."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            # setcbreak: reads keys immediately without Enter, but unlike
            # setraw it keeps output processing ON so \n still moves to
            # column 0.  setraw was causing all print() output to cascade
            # sideways because \n stopped adding a carriage return.
            tty.setcbreak(fd)
            while not self._quit.is_set():
                # select with a 0.1s timeout so this loop wakes up regularly
                # to check _quit rather than blocking forever on read(1).
                # Without this, stop() can set _quit but the thread stays
                # stuck waiting for a keypress, the finally block never runs,
                # and the terminal stays in cbreak mode after the script ends.
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue  # timeout -- check _quit and loop
                ch = sys.stdin.read(1)
                if ch in self.QUIT_KEYS:
                    self._quit.set()
                    break
                elif ch in self.PAUSE_KEYS:
                    if self._paused.is_set():
                        self._paused.clear()
                        sys.stdout.write("\r\033[K[PLAYING]  Resuming...\n")
                        sys.stdout.flush()
                    else:
                        self._paused.set()
                        sys.stdout.write(
                            "\r\033[K[PAUSED ]  Press Space or p to resume, q to quit.\n"
                        )
                        sys.stdout.flush()
        finally:
            # Always restore terminal -- thread exits cleanly via select
            # timeout so this finally block is guaranteed to run.
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# =============================================================================
# INTERRUPTIBLE SLEEP
#
# Replaces bare time.sleep() in the replay loop.  Breaks the wait into 50ms
# ticks so the loop can react to pause/quit quickly.
#
# PAUSE BEHAVIOUR:
#   When paused, the deadline is extended by the exact pause duration, so the
#   remaining wait after resuming is the same as it would have been without the
#   pause.  Example: 10s wait, paused after 3s for 30s -> resumes with 7s left.
# =============================================================================


def interruptible_sleep(seconds: float, controller: KeyboardController) -> bool:
    """
    Sleep for `seconds`, honouring pause and quit from KeyboardController.

    Returns True  if the full sleep completed (or was paused + resumed).
    Returns False if quit was requested during the sleep.
    """
    TICK = 0.05  # seconds between controller checks

    deadline = time.monotonic() + seconds

    while time.monotonic() < deadline:
        if controller.is_quit_requested():
            return False

        if controller.is_paused():
            pause_start = time.monotonic()
            resumed = controller.wait_while_paused()
            if not resumed:
                return False
            # Extend deadline by the time spent paused.
            deadline += time.monotonic() - pause_start

        remaining = deadline - time.monotonic()
        time.sleep(min(TICK, max(0.0, remaining)))

    return True


# =============================================================================
# COT XML BUILDER
# =============================================================================


def build_cot_message(
    robot_name: str,
    lat: float,
    lon: float,
    timestamp: datetime,
    course: float | None,
    track_speed: float,
    uid: str | None = None,
) -> tuple[str, str]:
    """
    Assemble a CoT b-m-p-s-m XML string for a single breadcrumb.

    Returns (xml_string, uid) so the caller can store the UID for later
    deletion via a t-x-d-d delete CoT.

    uid may be passed in (for deterministic replay) or left as None to
    generate a fresh uuid4.  Either way the used uid is returned as the
    second element of the tuple.

    The <track> element is only included when course is not None.
    """
    stale = timestamp + timedelta(days=365)
    time_str = timestamp.strftime("%Y-%m-%dT%H:%M:%SZ")
    stale_str = stale.strftime("%Y-%m-%dT%H:%M:%SZ")
    used_uid = uid if uid is not None else str(uuid.uuid4())

    track_element = (
        f'<track speed="{track_speed:.1f}" course="{course:.1f}"/>'
        if course is not None
        else ""
    )

    xml = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        f'<event version="2.0"'
        f' uid="{used_uid}"'
        f' type="b-m-p-s-m"'
        f' how="h-g-i-g-o"'
        f' time="{time_str}"'
        f' start="{time_str}"'
        f' stale="{stale_str}"'
        f' access="Undefined">'
        f'<point lat="{lat:.5f}" lon="{lon:.5f}" hae="0.0" ce="10.0" le="10.0"/>'
        f"<detail>"
        f'<precisionlocation geopointsrc="GPS" altsrc="GPS"/>'
        f'<status readiness="true"/>'
        f"<archive/>"
        f'<creator uid="{robot_name}" callsign="{robot_name}"'
        f' time="{time_str}" type="a-f-G-E-V"/>'
        f'<usericon iconsetpath="COT_MAPPING_SPOTMAP/b-m-p-s-m/-1"/>'
        f'<color argb="-1"/>'
        f'<link uid="{robot_name}" production_time="{time_str}"'
        f' type="a-f-G-E-V" parent_callsign="{robot_name}" relation="p-p"/>'
        f"{track_element}"
        f"<remarks/>"
        f"</detail>"
        f"</event>"
    )
    return xml, used_uid


# =============================================================================
# HOME COT BUILDER
#
# Produces a CoT a-u-G (unknown ground point) marker for the robot's home
# position.  Differences from the breadcrumb CoT:
#
#   - type        : a-u-G  (ground point, not a spot-map trail marker)
#   - uid         : deterministic UUID derived from robot_name so the same
#                   marker is updated in place on repeated replays rather than
#                   creating duplicate home icons on ATAK
#   - callsign    : {ROBOT_NAME}_HOME  (e.g. WARTHOG1_HOME)
#   - ce / le     : 9999999.0  (unknown accuracy -- this is a manually set pos)
#   - hae         : 0.0
#   - usericon    : house icon from the example CoT
#   - no <track>  : home is a static point, not a moving vehicle
#   - stale       : 1 year (keeps it on ATAK indefinitely)
# =============================================================================


def build_home_cot_message(robot_name: str, lat: float, lon: float) -> str:
    """
    Build a CoT a-u-G marker for the robot's home position.

    Uses a deterministic UUID (uuid5) so the same robot always produces the
    same UID -- ATAK will update the existing home marker rather than stacking
    duplicate icons on repeated replays.
    """
    now = datetime.now(timezone.utc)
    stale = now + timedelta(days=365)
    time_str = now.strftime("%Y-%m-%dT%H:%M:%SZ")
    stale_str = stale.strftime("%Y-%m-%dT%H:%M:%SZ")

    # Deterministic UID: same robot always produces the same home marker UID.
    home_uid = str(
        uuid.uuid5(uuid.NAMESPACE_URL, f"mission_database/{robot_name}/home")
    )

    callsign = f"{robot_name.upper()}_HOME"

    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        f'<event version="2.0"'
        f' uid="{home_uid}"'
        f' type="a-u-G"'
        f' how="h-g-i-g-o"'
        f' time="{time_str}"'
        f' start="{time_str}"'
        f' stale="{stale_str}"'
        f' access="Undefined">'
        f'<point lat="{lat:.7f}" lon="{lon:.7f}" hae="0.0"'
        f' ce="9999999.0" le="9999999.0"/>'
        f"<detail>"
        f'<contact callsign="{callsign}"/>'
        f'<status readiness="true"/>'
        f"<archive/>"
        f'<color argb="-1"/>'
        f'<usericon iconsetpath="ad78aafb-83a6-4c07-b2b9-a897a8b6a38f/Shapes/homegardenbusiness.png"/>'
        f'<link uid="{robot_name}" production_time="{time_str}"'
        f' type="a-f-G-U-C" parent_callsign="{robot_name}" relation="p-p"/>'
        f'<precisionlocation altsrc="SRTM1"/>'
        f"<remarks/>"
        f"</detail>"
        f"</event>"
    )


# =============================================================================
# DELETE COT BUILDER
#
# Produces a CoT t-x-d-d (delete) command that removes a specific symbol from
# ATAK.  The link uid must match the uid of the CoT that was originally sent,
# and the lat/lon must match the original point.
# =============================================================================


def build_delete_cot_message(target_uid: str, lat: float, lon: float) -> str:
    """
    Build a CoT t-x-d-d delete command targeting a specific UID.

    Parameters
    ----------
    target_uid : the uid of the CoT symbol to delete (breadcrumb or home)
    lat / lon  : coordinates of the original symbol
    """
    now = datetime.now(timezone.utc)
    stale = now + timedelta(minutes=1)
    time_str = now.strftime("%Y-%m-%dT%H:%M:%SZ")
    stale_str = stale.strftime("%Y-%m-%dT%H:%M:%SZ")
    cmd_uid = f"delete-cmd-{str(uuid.uuid4())[:8]}"

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
        f"<detail>"
        f'<link uid="{target_uid}" relation="none" type="none"/>'
        f"<__forcedelete/>"
        f"</detail>"
        f"</event>"
    )


def parse_db_timestamp(ts_str: str) -> datetime:
    """Parse '2024-03-15 14:23:07.042 UTC' to a UTC-aware datetime."""
    clean = ts_str.replace(" UTC", "").strip()
    for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.strptime(clean, fmt).replace(tzinfo=timezone.utc)
        except ValueError:
            continue
    raise ValueError(f"Unrecognised timestamp format: {ts_str!r}")


def infer_robot_name(db_path: Path) -> str:
    """
    Extract the robot name from a timestamped database filename.
    'warthog1_2024-03-15_14-23-07.db' -> 'warthog1'
    """
    parts = db_path.stem.split("_")
    return parts[0] if parts else db_path.stem


def load_home_position(db_path: Path) -> dict | None:
    """
    Load the stored home position from the database.
    Returns a dict with keys lat, lon, heading (may be None), set_by, timestamp.
    Returns None if no home position has been set.
    """
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    row = conn.execute(
        "SELECT lat, lon, heading, timestamp, set_by FROM home_position WHERE id=1"
    ).fetchone()
    conn.close()

    if row is None:
        return None

    return {
        "lat": row["lat"],
        "lon": row["lon"],
        "heading": float(row["heading"]) if row["heading"] is not None else None,
        "timestamp": row["timestamp"] or "",
        "set_by": row["set_by"] or "",
    }


def load_breadcrumbs(db_path: Path) -> list[dict]:
    """Load all breadcrumbs ordered chronologically, skipping unparseable rows."""
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    rows = conn.execute(
        "SELECT lat, lon, timestamp, heading FROM breadcrumbs ORDER BY id ASC"
    ).fetchall()
    conn.close()

    crumbs = []
    skipped = 0
    for row in rows:
        if not row["timestamp"]:
            skipped += 1
            continue
        try:
            ts = parse_db_timestamp(row["timestamp"])
        except ValueError as exc:
            print(f"  WARNING: skipping breadcrumb -- {exc}", file=sys.stderr)
            skipped += 1
            continue
        crumbs.append(
            {
                "lat": row["lat"],
                "lon": row["lon"],
                "ts": ts,
                # None preserved -- omits <track> element in CoT
                "heading": (
                    float(row["heading"]) if row["heading"] is not None else None
                ),
            }
        )

    if skipped:
        print(
            f"  WARNING: {skipped} breadcrumb(s) skipped (bad/missing timestamp).",
            file=sys.stderr,
        )
    return crumbs


# =============================================================================
# ROS2 NODE
# =============================================================================


class ReplayNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f"db_tak_replay_{robot_name}")
        self._topic = f"/{robot_name}/send_to_tak"
        self._pub = self.create_publisher(String, self._topic, qos_profile=10)
        self.get_logger().info(f"[db_tak_replay] Publishing CoT to: {self._topic}")

    def publish(self, xml: str) -> None:
        msg = String()
        msg.data = xml
        self._pub.publish(msg)


# =============================================================================
# REPLAY LOOP
# =============================================================================


def run_replay(
    node: ReplayNode,
    breadcrumbs: list[dict],
    robot_name: str,
    track_speed: float,
    speed_multiplier: float,
    instant: bool,
    controller: KeyboardController,
) -> list[dict]:
    """
    Replay breadcrumbs to TAK.

    Returns a list of dicts for every breadcrumb actually sent:
        [{'uid': <str>, 'lat': <float>, 'lon': <float>}, ...]
    This is used by the post-replay delete prompt to know which UIDs
    to send delete CoTs for.  If the user quits early, only the sent
    breadcrumbs are returned (not the full list).
    """
    total = len(breadcrumbs)
    sent_items = []  # accumulates {'uid', 'lat', 'lon'} for each sent crumb

    for i, crumb in enumerate(breadcrumbs):

        if controller.is_quit_requested():
            print("\nQuit requested -- stopping replay.")
            break

        # ── SEND COT ──────────────────────────────────────────────────────────
        cot_xml, cot_uid = build_cot_message(
            robot_name=robot_name,
            lat=crumb["lat"],
            lon=crumb["lon"],
            timestamp=crumb["ts"],
            course=crumb["heading"],
            track_speed=track_speed,
        )
        node.publish(cot_xml)
        sent_items.append({"uid": cot_uid, "lat": crumb["lat"], "lon": crumb["lon"]})

        # ── STATUS LINE ───────────────────────────────────────────────────────
        hdg_str = (
            f'{crumb["heading"]:.1f}deg' if crumb["heading"] is not None else "n/a"
        )
        status_tag = "[PAUSED ]" if controller.is_paused() else "[PLAYING]"
        print(
            f"{status_tag} [{i+1:>4}/{total}]  "
            f'{crumb["ts"].strftime("%Y-%m-%d %H:%M:%S")} UTC  '
            f'({crumb["lat"]:.5f}, {crumb["lon"]:.5f})  hdg={hdg_str}'
        )

        # ── TIMING ────────────────────────────────────────────────────────────
        if instant:
            # Tiny yield so DDS dispatches each message before the next.
            rclpy.spin_once(node, timeout_sec=0.01)

        elif i < total - 1:
            gap_secs = (breadcrumbs[i + 1]["ts"] - crumb["ts"]).total_seconds()

            if gap_secs < 0:
                print(
                    f"  WARNING: negative time gap ({gap_secs:.1f}s) -- sending immediately.",
                    file=sys.stderr,
                )
                gap_secs = 0.0

            sleep_secs = gap_secs / speed_multiplier

            if sleep_secs > 0:
                print(
                    f"  Next in {sleep_secs:.1f}s "
                    f"(real gap: {gap_secs:.1f}s  speed: {speed_multiplier}x) "
                    f" [Space/p = pause  q = quit]"
                )
                completed = interruptible_sleep(sleep_secs, controller)
                if not completed:
                    print("\nQuit requested during wait -- stopping replay.")
                    break

            rclpy.spin_once(node, timeout_sec=0.0)

    return sent_items


# =============================================================================
# MAIN
# =============================================================================


def main() -> None:

    parser = argparse.ArgumentParser(
        prog="db_tak_replay",
        description="Replay a mission_database SQLite file to TAK via ROS2.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "TERMINAL CONTROLS (timed mode)\n"
            "  Space / p  -- pause / resume\n"
            "  q          -- quit\n\n"
            "EXAMPLES\n"
            "  python3 db_tak_replay.py warthog1_2024-03-15_14-23-07.db\n"
            "  python3 db_tak_replay.py warthog1_2024-03-15_14-23-07.db --speed 4.0\n"
            "  python3 db_tak_replay.py warthog1_2024-03-15_14-23-07.db --instant\n"
        ),
    )

    parser.add_argument(
        "db_path", help="Path to the mission_database .db file to replay."
    )
    parser.add_argument(
        "--robot_name",
        default=None,
        metavar="NAME",
        help="Robot name for the TAK topic and CoT callsign. "
        "Inferred from filename if not specified.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        metavar="MULTIPLIER",
        help="Playback speed multiplier (default: 1.0). Ignored with --instant.",
    )
    parser.add_argument(
        "--instant",
        action="store_true",
        help="Send all breadcrumbs immediately with no timing delays.",
    )
    parser.add_argument(
        "--track_speed",
        type=float,
        default=1.4,
        metavar="M_PER_S",
        help='CoT <track speed="..."> value in m/s (default: 1.4).',
    )
    parser.add_argument(
        "--skip_home",
        action="store_true",
        help="Do not send the home position CoT marker before replaying breadcrumbs.",
    )

    args = parser.parse_args()

    # ── VALIDATE ──────────────────────────────────────────────────────────────

    db_path = Path(args.db_path)
    if not db_path.exists():
        print(f"ERROR: Database file not found: {db_path}", file=sys.stderr)
        sys.exit(1)
    if args.speed <= 0.0:
        print(f"ERROR: --speed must be > 0 (got {args.speed})", file=sys.stderr)
        sys.exit(1)

    robot_name = args.robot_name or infer_robot_name(db_path)

    # ── LOAD ──────────────────────────────────────────────────────────────────

    print(f"\nDB TAK Replay")
    print(f"  Database    : {db_path}")
    print(f"  Robot       : {robot_name}")
    print(
        f'  Mode        : {"INSTANT" if args.instant else f"{args.speed}x real-time"}'
    )
    print(f"  Track speed : {args.track_speed} m/s")
    print()

    print("Loading breadcrumbs...")
    breadcrumbs = load_breadcrumbs(db_path)
    if not breadcrumbs:
        print("No valid breadcrumbs found. Nothing to replay.")
        sys.exit(0)

    first_ts = breadcrumbs[0]["ts"]
    last_ts = breadcrumbs[-1]["ts"]
    duration = (last_ts - first_ts).total_seconds()

    print(f"  {len(breadcrumbs)} breadcrumbs loaded")
    print(f'  Mission start : {first_ts.strftime("%Y-%m-%d %H:%M:%S")} UTC')
    print(f'  Mission end   : {last_ts.strftime("%Y-%m-%d %H:%M:%S")} UTC')
    print(f"  Duration      : {duration:.0f}s  ({duration / 60:.1f} min)")
    if not args.instant:
        replay_secs = duration / args.speed
        print(
            f"  Replay time   : ~{replay_secs:.0f}s  ({replay_secs / 60:.1f} min) at {args.speed}x"
        )
    print()

    # ── KEYBOARD CONTROLLER ───────────────────────────────────────────────────

    controller = KeyboardController()

    if not args.instant:
        tty_ok = controller.start()
        if tty_ok:
            print("Controls: Space/p = pause/resume   q = quit")
        else:
            print("(Stdin is not a TTY -- pause/resume controls unavailable)")
        print()
    else:
        # Start controller anyway so quit-detection via Ctrl+C still works.
        controller.start()

    # ── ROS2 INIT ─────────────────────────────────────────────────────────────

    rclpy.init()
    node = ReplayNode(robot_name)

    print("Waiting for subscriber connection...")
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    print("Ready.\n")

    # ── HOME COT ──────────────────────────────────────────────────────────────

    try:
        if not args.skip_home:
            home = load_home_position(db_path)
            if home is not None:
                print(
                    f'Sending home position CoT: ({home["lat"]:.7f}, {home["lon"]:.7f})'
                    f'  set_by={home["set_by"]}'
                )
                home_xml = build_home_cot_message(robot_name, home["lat"], home["lon"])
                node.publish(home_xml)
                rclpy.spin_once(node, timeout_sec=0.1)
                print(f"  Callsign : {robot_name.upper()}_HOME")
                print()
            else:
                print("No home position stored in database -- skipping home CoT.\n")
        else:
            print("Home CoT skipped (--skip_home).\n")
            home = None

        # ── REPLAY ────────────────────────────────────────────────────────────────

        print(f"Starting replay -- {len(breadcrumbs)} breadcrumbs.\n")
        sent_items = run_replay(
            node=node,
            breadcrumbs=breadcrumbs,
            robot_name=robot_name,
            track_speed=args.track_speed,
            speed_multiplier=args.speed,
            instant=args.instant,
            controller=controller,
        )

        replay_complete = not controller.is_quit_requested()
        if replay_complete:
            print(f"\nReplay complete.  {len(sent_items)} breadcrumbs sent.")

        # ── STOP KEYBOARD CONTROLLER before input() ────────────────────────────────
        # join() waits for the select loop to exit (within ~0.1s) and for the
        # finally block to restore the terminal.  Must happen before input().

        controller.stop()

        # ── DELETE PROMPT ──────────────────────────────────────────────────────────
        # Use replay_complete (captured before stop()) not is_quit_requested()
        # which would always be True after stop() sets the quit flag.

        if sent_items and replay_complete:
            home_uid = (
                str(
                    uuid.uuid5(
                        uuid.NAMESPACE_URL, f"mission_database/{robot_name}/home"
                    )
                )
                if (home is not None and not args.skip_home)
                else None
            )
            total_symbols = len(sent_items) + (1 if home_uid else 0)

            print(
                f"\nDelete {total_symbols} symbol(s) from ATAK? "
                f"({len(sent_items)} breadcrumbs"
                f'{" + 1 home marker" if home_uid else ""}) [y/N] ',
                end="",
                flush=True,
            )

            try:
                answer = input().strip().lower()
            except EOFError:
                answer = "n"

            if answer == "y":
                print(f"Sending {total_symbols} delete CoT(s)...")

                # Delete breadcrumbs
                for item in sent_items:
                    del_xml = build_delete_cot_message(
                        item["uid"], item["lat"], item["lon"]
                    )
                    node.publish(del_xml)
                    rclpy.spin_once(node, timeout_sec=0.01)

                # Delete home marker
                if home_uid and home is not None:
                    del_xml = build_delete_cot_message(
                        home_uid, home["lat"], home["lon"]
                    )
                    node.publish(del_xml)
                    rclpy.spin_once(node, timeout_sec=0.1)

                print("Delete commands sent.")
            else:
                print("Symbols kept on ATAK.")

    except KeyboardInterrupt:
        controller.request_quit()
        print("\nInterrupted.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
