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

    PAUSE_KEYS = {' ', 'p', 'P'}
    QUIT_KEYS  = {'q', 'Q'}   # Ctrl+C is SIGINT via setcbreak, caught by KeyboardInterrupt

    def __init__(self):
        self._paused      = threading.Event()
        self._quit        = threading.Event()
        self._tty_active  = False
        self._thread      = threading.Thread(
            target=self._read_keys, daemon=True, name='kb_controller'
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
                    continue   # timeout -- check _quit and loop
                ch = sys.stdin.read(1)
                if ch in self.QUIT_KEYS:
                    self._quit.set()
                    break
                elif ch in self.PAUSE_KEYS:
                    if self._paused.is_set():
                        self._paused.clear()
                        sys.stdout.write('\r\033[K[PLAYING]  Resuming...\n')
                        sys.stdout.flush()
                    else:
                        self._paused.set()
                        sys.stdout.write('\r\033[K[PAUSED ]  Press Space or p to resume, q to quit.\n')
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
    robot_name:  str,
    lat:         float,
    lon:         float,
    timestamp:   datetime,
    course:      float | None,
    track_speed: float,
    uid:         str | None = None,
    color_argb:  str = '-1',
) -> tuple[str, str]:
    """
    Assemble a CoT b-m-p-s-m XML string for a single breadcrumb.

    Returns (xml_string, uid) so the caller can store the UID for later
    deletion via a t-x-d-d delete CoT.

    uid may be passed in (for deterministic replay) or left as None to
    generate a fresh uuid4.  Either way the used uid is returned as the
    second element of the tuple.

    The <track> element is only included when course is not None.

    color_argb controls both the spot icon and the <color> element:
        '-1'      -- white  (default, comms OK or unknown)
        '-65536'  -- red    (comms lost)
    ATAK uses the iconsetpath suffix to pick the coloured spot variant and
    the <color> element to tint the label.  Both must match for a consistent
    appearance.
    """
    stale     = timestamp + timedelta(days=365)
    time_str  = timestamp.strftime('%Y-%m-%dT%H:%M:%SZ')
    stale_str = stale.strftime('%Y-%m-%dT%H:%M:%SZ')
    used_uid  = uid if uid is not None else str(uuid.uuid4())

    track_element = (
        f'<track speed="{track_speed:.1f}" course="{course:.1f}"/>'
        if course is not None
        else ''
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
        f'<detail>'
        f'<precisionlocation geopointsrc="GPS" altsrc="GPS"/>'
        f'<status readiness="true"/>'
        f'<archive/>'
        f'<creator uid="{robot_name}" callsign="{robot_name}"'
        f' time="{time_str}" type="a-f-G-E-V"/>'
        f'<usericon iconsetpath="COT_MAPPING_SPOTMAP/b-m-p-s-m/{color_argb}"/>'
        f'<color argb="{color_argb}"/>'
        f'<link uid="{robot_name}" production_time="{time_str}"'
        f' type="a-f-G-E-V" parent_callsign="{robot_name}" relation="p-p"/>'
        f'{track_element}'
        f'<remarks/>'
        f'</detail>'
        f'</event>'
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
    now       = datetime.now(timezone.utc)
    stale     = now + timedelta(days=365)
    time_str  = now.strftime('%Y-%m-%dT%H:%M:%SZ')
    stale_str = stale.strftime('%Y-%m-%dT%H:%M:%SZ')

    # Deterministic UID: same robot always produces the same home marker UID.
    home_uid  = str(uuid.uuid5(uuid.NAMESPACE_URL,
                               f'mission_database/{robot_name}/home'))

    callsign  = f'{robot_name.upper()}_HOME'

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
        f'<detail>'
        f'<contact callsign="{callsign}"/>'
        f'<status readiness="true"/>'
        f'<archive/>'
        f'<color argb="-1"/>'
        f'<usericon iconsetpath="ad78aafb-83a6-4c07-b2b9-a897a8b6a38f/Shapes/homegardenbusiness.png"/>'
        f'<link uid="{robot_name}" production_time="{time_str}"'
        f' type="a-f-G-U-C" parent_callsign="{robot_name}" relation="p-p"/>'
        f'<precisionlocation altsrc="SRTM1"/>'
        f'<remarks/>'
        f'</detail>'
        f'</event>'
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
# WAYPOINT CIRCLE COT BUILDER
#
# Produces a CoT u-d-c-c (drawing circle) marker for a waypoint.
# Colour is determined by waypoint status:
#   pending  -> yellow  (DISPATCHED, not yet completed)
#   reached  -> green   (navigation succeeded)
#   failed   -> red     (navigation failed or aborted)
#
# The UID is deterministic (uuid5) so the same waypoint always produces the
# same UID.  When a waypoint transitions from pending -> reached/failed, the
# replay sends a new circle with the updated colour and the same UID, which
# causes ATAK to replace the marker in-place rather than stacking duplicates.
#
# Callsign is "WP {waypt_num}" so it's human-readable on the ATAK display.
# Radius is stored per-waypoint in the DB (default 2.0 m).
# =============================================================================

# Colour definitions extracted from the TAK CoT examples.
# KML LineStyle/PolyStyle colours are AARRGGBB.
# strokeColor / fillColor are signed 32-bit ARGB integers (Java convention).
_WP_COLOURS = {
    'pending': {
        'line_kml':   'ffffff00',   # opaque yellow
        'fill_kml':   '1cffff00',   # 11% yellow
        'stroke_val': '-256',
        'fill_val':   '486539008',
    },
    'reached': {
        'line_kml':   'ff00ff00',   # opaque green
        'fill_kml':   '1c00ff00',   # 11% green
        'stroke_val': '-16711936',
        'fill_val':   '469827328',
    },
    'failed': {
        'line_kml':   'ffff0000',   # opaque red
        'fill_kml':   '1cff0000',   # 11% red
        'stroke_val': '-65536',
        'fill_val':   '486473728',
    },
}


def build_waypoint_circle_cot(
    robot_name:  str,
    waypoint_id: int,
    lat:         float,
    lon:         float,
    radius:      float,
    name:        str,
    status:      str,
) -> str:
    """
    Build a CoT u-d-c-c (drawing circle) for a waypoint.

    Parameters
    ----------
    robot_name  : used to make the UID deterministic per robot+waypoint
    waypoint_id : numeric ID, used in callsign and UID derivation
    lat / lon   : waypoint centre coordinates
    radius      : circle radius in metres (stored in the DB, default 2.0)
    name        : human-readable waypoint label (appended to callsign if set)
    status      : 'pending' | 'reached' | 'failed'
    """
    now       = datetime.now(timezone.utc)
    stale     = now + timedelta(days=365)
    time_str  = now.strftime('%Y-%m-%dT%H:%M:%SZ')
    stale_str = stale.strftime('%Y-%m-%dT%H:%M:%SZ')

    # Deterministic UID: same robot+waypoint always produces the same UID so
    # ATAK updates the existing marker when status changes (yellow -> green/red)
    # rather than creating a new duplicate circle.
    wp_uid = str(uuid.uuid5(uuid.NAMESPACE_URL,
                             f'mission_database/{robot_name}/waypoint/{waypoint_id}'))

    # Callsign: "WP3" when unnamed, "WP3: checkpoint_alpha" when named.
    # No space between WP and the number so the label stays compact on ATAK.
    callsign = f'WP{waypoint_id}'
    if name:
        callsign += f': {name}'

    # Normalise status -- any unrecognised value falls back to pending (yellow)
    colours = _WP_COLOURS.get(status, _WP_COLOURS['pending'])

    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        f'<event version="2.0"'
        f' uid="{wp_uid}"'
        f' type="u-d-c-c"'
        f' how="h-e"'
        f' time="{time_str}"'
        f' start="{time_str}"'
        f' stale="{stale_str}"'
        f' access="Undefined">'
        f'<point lat="{lat:.7f}" lon="{lon:.7f}" hae="0.0"'
        f' ce="9999999.0" le="9999999.0"/>'
        f'<detail>'
        f'<contact callsign="{callsign}"/>'
        f'<shape>'
        f'<ellipse major="{radius:.1f}" minor="{radius:.1f}" angle="360"/>'
        f'<link uid="{wp_uid}.Style" type="b-x-KmlStyle" relation="p-c">'
        f'<Style>'
        f'<LineStyle>'
        f'<color>{colours["line_kml"]}</color>'
        f'<width>1.0</width>'
        f'</LineStyle>'
        f'<PolyStyle>'
        f'<color>{colours["fill_kml"]}</color>'
        f'</PolyStyle>'
        f'</Style>'
        f'</link>'
        f'</shape>'
        f'<__shapeExtras cpvis="true" editable="true"/>'
        f'<archive/>'
        f'<strokeColor value="{colours["stroke_val"]}"/>'
        f'<strokeWeight value="1.0"/>'
        f'<strokeStyle value="dotted"/>'
        f'<fillColor value="{colours["fill_val"]}"/>'
        f'<labels_on value="true"/>'
        f'<precisionlocation altsrc="SRTM1"/>'
        f'<remarks/>'
        f'</detail>'
        f'</event>'
    )

def parse_db_timestamp(ts_str: str) -> datetime:
    """Parse '2024-03-15 14:23:07.042 UTC' to a UTC-aware datetime."""
    clean = ts_str.replace(' UTC', '').strip()
    for fmt in ('%Y-%m-%d %H:%M:%S.%f', '%Y-%m-%d %H:%M:%S'):
        try:
            return datetime.strptime(clean, fmt).replace(tzinfo=timezone.utc)
        except ValueError:
            continue
    raise ValueError(f'Unrecognised timestamp format: {ts_str!r}')


def infer_robot_name(db_path: Path) -> str:
    """
    Extract the robot name from a timestamped database filename.
    'warthog1_2024-03-15_14-23-07.db' -> 'warthog1'
    """
    parts = db_path.stem.split('_')
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
        'SELECT lat, lon, heading, timestamp, set_by FROM home_position WHERE id=1'
    ).fetchone()
    conn.close()

    if row is None:
        return None

    return {
        'lat':       row['lat'],
        'lon':       row['lon'],
        'heading':   float(row['heading']) if row['heading'] is not None else None,
        'timestamp': row['timestamp'] or '',
        'set_by':    row['set_by']    or '',
    }


def load_waypoints(db_path: Path) -> list[dict]:
    """
    Load all waypoints from the database ordered by dispatch time.
    Returns a list of dicts with keys:
        waypoint_id, lat, lon, radius, name, status, dispatched_at, completed_at
    All statuses are included (pending, reached, failed) so the replay can
    show the correct circle colour for each.
    """
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    rows = conn.execute(
        'SELECT waypoint_id, lat, lon, radius, name, status, '
        '       dispatched_at, completed_at '
        'FROM waypoints ORDER BY id ASC'
    ).fetchall()
    conn.close()

    return [{
        'waypoint_id':    row['waypoint_id'],
        'lat':            row['lat'],
        'lon':            row['lon'],
        'radius':         float(row['radius']) if row['radius'] is not None else 2.0,
        'name':           row['name'] or '',
        'status':         row['status'] or 'pending',
        'dispatched_at':  row['dispatched_at'] or '',
        'completed_at':   row['completed_at'] or '',
        # Pre-parsed timestamps for timeline comparison during replay.
        'dispatched_ts':  _parse_waypoint_ts(row['dispatched_at']),
        'completed_ts':   _parse_waypoint_ts(row['completed_at']),
    } for row in rows]


def _parse_waypoint_ts(ts_str: str | None) -> datetime | None:
    """Parse a waypoint timestamp string to a UTC datetime, or None if absent."""
    if not ts_str:
        return None
    try:
        return parse_db_timestamp(ts_str)
    except ValueError:
        return None


def _detect_comms_column(conn: sqlite3.Connection) -> str | None:
    """
    Scan the breadcrumbs table schema for a column that records comms status.

    The confirmed column name from mission_database_node.cpp is 'has_comms'
    (INTEGER: NULL=unknown, 0=no, 1=yes).  Additional candidates are kept for
    forward compatibility if the schema is ever extended or renamed.
    """
    _COMMS_CANDIDATES = (
        'has_comms',        # mission_database_node.cpp: NULL=unknown, 0=no, 1=yes
        'comms_connected',
        'base_reachable',
        'comms_ok',
        'comms_status',
    )
    rows = conn.execute("PRAGMA table_info(breadcrumbs)").fetchall()
    existing = {row[1].lower() for row in rows}   # row[1] = column name
    for candidate in _COMMS_CANDIDATES:
        if candidate in existing:
            return candidate
    return None


def _parse_comms_value(raw) -> bool | None:
    """
    Convert a raw DB value from the comms column to a bool.

    Handles the confirmed has_comms schema (INTEGER NULL=unknown, 0=no, 1=yes)
    plus text variants for forward compatibility:
      - None / SQL NULL   -> None   (unknown -- column present but not recorded)
      - 1 / True          -> True   (comms up)
      - 0 / False         -> False  (comms lost)
      - Text 'connected' / 'ok' / 'true' / '1'            -> True
      - Text 'disconnected' / 'lost' / 'false' / '0' etc. -> False
      - Anything else     -> None   (unrecognised -- treat as unknown)
    """
    if raw is None:
        return None
    if isinstance(raw, (int, float)):
        return bool(raw)
    s = str(raw).strip().lower()
    if s in ('1', 'true', 'connected', 'ok', 'good', 'up'):
        return True
    if s in ('0', 'false', 'disconnected', 'lost', 'bad', 'down', 'degraded', 'no_link'):
        return False
    return None   # unrecognised text -- treat as unknown


def load_breadcrumbs(db_path: Path) -> list[dict]:
    """
    Load all breadcrumbs ordered chronologically, skipping unparseable rows.

    Also loads comms status per breadcrumb when a recognised comms column is
    present in the schema (see _detect_comms_column for candidates).  Each
    breadcrumb gets a 'comms_ok' key:
        True   -- link confirmed up at this timestamp
        False  -- link confirmed down at this timestamp
        None   -- column absent in this DB, or value not recorded
    """
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row

    # Detect which comms column (if any) exists in this database's schema.
    comms_col = _detect_comms_column(conn)

    select_cols = 'lat, lon, timestamp, heading, speed'
    if comms_col:
        select_cols += f', {comms_col}'

    rows = conn.execute(
        f'SELECT {select_cols} FROM breadcrumbs ORDER BY id ASC'
    ).fetchall()
    conn.close()

    crumbs  = []
    skipped = 0
    for row in rows:
        if not row['timestamp']:
            skipped += 1
            continue
        try:
            ts = parse_db_timestamp(row['timestamp'])
        except ValueError as exc:
            print(f'  WARNING: skipping breadcrumb -- {exc}', file=sys.stderr)
            skipped += 1
            continue

        comms_ok = _parse_comms_value(row[comms_col]) if comms_col else None

        crumbs.append({
            'lat':      row['lat'],
            'lon':      row['lon'],
            'ts':       ts,
            # None preserved -- omits <track> element in CoT
            'heading':  float(row['heading']) if row['heading'] is not None else None,
            # None means gps_speed was not yet received when this crumb was recorded;
            # the replay will fall back to 0.0 (or --track_speed if overridden).
            'speed':    float(row['speed'])   if row['speed']   is not None else None,
            # True/False/None -- drives CoT color and terminal display.
            'comms_ok': comms_ok,
        })

    if skipped:
        print(f'  WARNING: {skipped} breadcrumb(s) skipped (bad/missing timestamp).',
              file=sys.stderr)
    return crumbs

    if skipped:
        print(f'  WARNING: {skipped} breadcrumb(s) skipped (bad/missing timestamp).',
              file=sys.stderr)
    return crumbs


# =============================================================================
# ROS2 NODE
# =============================================================================

class ReplayNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f'db_tak_replay_{robot_name}')
        self._topic = f'/{robot_name}/send_to_tak'
        self._pub   = self.create_publisher(String, self._topic, qos_profile=10)
        self.get_logger().info(f'[db_tak_replay] Publishing CoT to: {self._topic}')

    def publish(self, xml: str) -> None:
        msg      = String()
        msg.data = xml
        self._pub.publish(msg)


def publish_with_retry(node: 'ReplayNode', xml: str,
                       retries: int = 5, delay: float = 0.1) -> None:
    """
    Publish a CoT XML string `retries` times with `delay` seconds between
    each send.  Redundant transmission guards against occasional message drops
    on the send_to_tak topic without requiring acknowledgement from the TAK
    server.

    spin_once uses a small non-zero timeout (0.01s) after each publish so the
    DDS middleware has a chance to actually dispatch the outgoing message before
    the next one is queued.  timeout_sec=0.0 returns immediately and can leave
    messages sitting in the queue unflushed.

    Parameters
    ----------
    node    : ReplayNode to publish through
    xml     : CoT XML string to send
    retries : number of times to send (default 5)
    delay   : seconds between sends (default 0.1 = 100 ms)
    """
    for _ in range(retries):
        node.publish(xml)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(delay)


# =============================================================================
# COMMS COLOR CONSTANTS
#
# CoT b-m-p-s-m spot-map icons are selected by appending the ARGB color
# value to the iconsetpath.  The same value goes into <color argb="...">.
#
#   -1      : white  (all bits set: 0xFFFFFFFF) -- comms OK or unknown
#   -65536  : red    (0xFFFF0000)               -- comms lost
# =============================================================================

_COMMS_COLOR_OK      = '-1'       # white
_COMMS_COLOR_LOST    = '-65536'   # red


def _comms_color(comms_ok: bool | None) -> str:
    """Return the ARGB color string for a breadcrumb's comms state."""
    if comms_ok is False:
        return _COMMS_COLOR_LOST
    return _COMMS_COLOR_OK          # True or None → white


def _comms_label(comms_ok: bool | None) -> str:
    """Return a short terminal label for a breadcrumb's comms state."""
    if comms_ok is True:
        return 'OK'
    if comms_ok is False:
        return 'LOST'
    return '?'


# =============================================================================
# REPLAY LOOP
#
# DESIGN: Unified event timeline
# --------------------------------
# Pre-builds a single sorted list of ALL events before the loop starts:
#
#   - 'breadcrumb'        : a GPS trail point → b-m-p-s-m CoT (white or red)
#   - 'waypoint_dispatch' : send yellow circle at dispatched_ts
#   - 'waypoint_complete' : send green/red circle at completed_ts
#
# Every event gets a proper [N/Total] counter so the operator can see exactly
# how many things are happening and in what order.
#
# TIMING: The inter-event gap drives interruptible_sleep regardless of event
# type, so timing is accurate across breadcrumbs and waypoint transitions.
#
# COMMS STATUS: Each breadcrumb CoT is coloured based on the comms_ok flag
# loaded from the database:
#   comms_ok=True  → white spot (normal)
#   comms_ok=False → red   spot (link lost at this timestamp)
#   comms_ok=None  → white spot (no comms data in this DB)
# The terminal status line shows  comms=OK / comms=LOST / comms=?  for each
# breadcrumb.  When the DB has no comms column the field is omitted entirely.
#
# POST-REPLAY WAYPOINTS: If completed_ts falls after the last breadcrumb,
# the completion event is appended at its real timestamp so it fires after a
# timed gap rather than as a silent post-loop flush.
# =============================================================================

def run_replay(
    node:             ReplayNode,
    breadcrumbs:      list[dict],
    waypoints:        list[dict],
    robot_name:       str,
    track_speed:      float,
    speed_multiplier: float,
    instant:          bool,
    controller:       KeyboardController,
) -> list[dict]:
    """
    Replay all events (breadcrumbs + waypoint transitions) to TAK in
    chronological order.

    Returns a list of dicts for every breadcrumb CoT actually sent --
    these are needed by the caller to issue delete CoTs at the end.
    Waypoint CoTs use deterministic UIDs handled separately in main().

        [{'uid': <str>, 'lat': <float>, 'lon': <float>}, ...]
    """

    # ── BUILD UNIFIED EVENT TIMELINE ──────────────────────────────────────────
    # Each entry is a dict:
    #   type='breadcrumb'        → keys: ts, crumb
    #   type='waypoint_dispatch' → keys: ts, wp
    #   type='waypoint_complete' → keys: ts, wp
    all_events: list[dict] = []

    for crumb in breadcrumbs:
        all_events.append({'type': 'breadcrumb', 'ts': crumb['ts'], 'crumb': crumb})

    last_breadcrumb_ts = breadcrumbs[-1]['ts'] if breadcrumbs else None

    for wp in waypoints:
        if wp['dispatched_ts'] is not None:
            all_events.append({
                'type': 'waypoint_dispatch',
                'ts':   wp['dispatched_ts'],
                'wp':   wp,
            })

        if wp['status'] in ('reached', 'failed'):
            if wp['completed_ts'] is not None:
                completion_ts = wp['completed_ts']
            elif last_breadcrumb_ts is not None:
                # Status is final but no timestamp recorded -- place just after
                # the last breadcrumb so it still appears in the event list.
                completion_ts = last_breadcrumb_ts + timedelta(milliseconds=500)
            else:
                completion_ts = wp['dispatched_ts']
            all_events.append({
                'type': 'waypoint_complete',
                'ts':   completion_ts,
                'wp':   wp,
            })

    # Sort chronologically.  Breadcrumbs win ties so the position that
    # triggered a waypoint transition appears before the transition itself.
    _TYPE_ORDER = {'breadcrumb': 0, 'waypoint_dispatch': 1, 'waypoint_complete': 2}
    all_events.sort(key=lambda e: (e['ts'], _TYPE_ORDER.get(e['type'], 9)))

    total = len(all_events)

    # Determine whether any comms data was loaded -- used to decide whether
    # to show the comms column in the terminal.
    has_comms_data = any(
        c.get('comms_ok') is not None for c in breadcrumbs
    )

    # Guard sets so each waypoint transition fires at most once.
    dispatched_wp_ids: set[int] = set()
    completed_wp_ids:  set[int] = set()

    sent_items:       list[dict] = []   # breadcrumb CoTs (needed for deletion)
    breadcrumb_count  = 0
    wp_event_count    = 0

    # ── MAIN EVENT LOOP ───────────────────────────────────────────────────────

    for i, event in enumerate(all_events):

        if controller.is_quit_requested():
            print('\nQuit requested -- stopping replay.')
            break

        status_tag = '[PAUSED ]' if controller.is_paused() else '[PLAYING]'
        ts_str     = event['ts'].strftime('%Y-%m-%d %H:%M:%S')

        # ────────────────────────────────────────────────────────────────────
        # BREADCRUMB EVENT
        # ────────────────────────────────────────────────────────────────────
        if event['type'] == 'breadcrumb':
            crumb = event['crumb']

            # Use recorded GPS speed when available; fall back to 0.0 (or
            # --track_speed if the user overrode it) for NULL-speed rows.
            effective_speed = (
                crumb['speed'] if crumb['speed'] is not None else track_speed
            )

            # Choose spot color: red when comms lost, white otherwise.
            color = _comms_color(crumb.get('comms_ok'))

            cot_xml, cot_uid = build_cot_message(
                robot_name  = robot_name,
                lat         = crumb['lat'],
                lon         = crumb['lon'],
                timestamp   = crumb['ts'],
                course      = crumb['heading'],
                track_speed = effective_speed,
                color_argb  = color,
            )
            publish_with_retry(node, cot_xml)
            sent_items.append({'uid': cot_uid,
                                'lat': crumb['lat'],
                                'lon': crumb['lon']})
            breadcrumb_count += 1

            hdg_str = (f'{crumb["heading"]:.1f}deg'
                       if crumb['heading'] is not None else 'n/a')
            # '*' suffix: speed was NULL in DB -- value is a fallback
            spd_str = (f'{effective_speed:.2f}m/s'
                       + ('' if crumb['speed'] is not None else '*'))

            # comms suffix: only shown when the DB has comms data
            comms_str = (f'  comms={_comms_label(crumb.get("comms_ok"))}'
                         if has_comms_data else '')

            print(
                f'{status_tag} [{i+1:>4}/{total}]  {ts_str} UTC  '
                f'({crumb["lat"]:.5f}, {crumb["lon"]:.5f})  '
                f'hdg={hdg_str}  spd={spd_str}{comms_str}'
            )

        # ────────────────────────────────────────────────────────────────────
        # WAYPOINT DISPATCH EVENT  (yellow circle)
        # ────────────────────────────────────────────────────────────────────
        elif event['type'] == 'waypoint_dispatch':
            wp    = event['wp']
            wp_id = wp['waypoint_id']

            if wp_id not in dispatched_wp_ids:
                wp_xml = build_waypoint_circle_cot(
                    robot_name  = robot_name,
                    waypoint_id = wp_id,
                    lat         = wp['lat'],
                    lon         = wp['lon'],
                    radius      = wp['radius'],
                    name        = wp['name'],
                    status      = 'pending',
                )
                publish_with_retry(node, wp_xml)
                dispatched_wp_ids.add(wp_id)
                wp_event_count += 1

            print(
                f'{status_tag} [{i+1:>4}/{total}]  {ts_str} UTC  '
                f'-> WP{wp_id} dispatched (yellow)'
            )

        # ────────────────────────────────────────────────────────────────────
        # WAYPOINT COMPLETION EVENT  (green / red circle)
        # ────────────────────────────────────────────────────────────────────
        elif event['type'] == 'waypoint_complete':
            wp    = event['wp']
            wp_id = wp['waypoint_id']

            if wp_id not in completed_wp_ids:
                wp_xml = build_waypoint_circle_cot(
                    robot_name  = robot_name,
                    waypoint_id = wp_id,
                    lat         = wp['lat'],
                    lon         = wp['lon'],
                    radius      = wp['radius'],
                    name        = wp['name'],
                    status      = wp['status'],
                )
                publish_with_retry(node, wp_xml)
                completed_wp_ids.add(wp_id)
                wp_event_count += 1

            colour = 'green' if wp['status'] == 'reached' else 'red'
            print(
                f'{status_tag} [{i+1:>4}/{total}]  {ts_str} UTC  '
                f'-> WP{wp_id} {wp["status"]} ({colour})'
            )

        # ── TIMING ────────────────────────────────────────────────────────────
        if instant:
            rclpy.spin_once(node, timeout_sec=0.01)

        elif i < total - 1:
            gap_secs = (all_events[i + 1]['ts'] - event['ts']).total_seconds()

            if gap_secs < 0:
                print(f'  WARNING: negative time gap ({gap_secs:.1f}s) -- sending immediately.',
                      file=sys.stderr)
                gap_secs = 0.0

            sleep_secs = gap_secs / speed_multiplier

            if sleep_secs > 0:
                print(
                    f'  Next in {sleep_secs:.1f}s '
                    f'(real gap: {gap_secs:.1f}s  speed: {speed_multiplier}x) '
                    f' [Space/p = pause  q = quit]'
                )
                completed = interruptible_sleep(sleep_secs, controller)
                if not completed:
                    print('\nQuit requested during wait -- stopping replay.')
                    break

            rclpy.spin_once(node, timeout_sec=0.0)

    # Store counts on node so main() can use them in the completion message.
    node._breadcrumb_count = breadcrumb_count
    node._wp_event_count   = wp_event_count

    return sent_items


# =============================================================================
# MAIN
# =============================================================================

def main() -> None:

    parser = argparse.ArgumentParser(
        prog='db_tak_replay',
        description='Replay a mission_database SQLite file to TAK via ROS2.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'TERMINAL CONTROLS (timed mode)\n'
            '  Space / p  -- pause / resume\n'
            '  q          -- quit\n\n'
            'EXAMPLES\n'
            '  python3 db_tak_replay.py warthog1_2024-03-15_14-23-07.db\n'
            '  python3 db_tak_replay.py warthog1_2024-03-15_14-23-07.db --speed 4.0\n'
            '  python3 db_tak_replay.py warthog1_2024-03-15_14-23-07.db --instant\n'
        ),
    )

    parser.add_argument('db_path',
        help='Path to the mission_database .db file to replay.')
    parser.add_argument('--robot_name', default=None, metavar='NAME',
        help='Robot name for the TAK topic and CoT callsign. '
             'Inferred from filename if not specified.')
    parser.add_argument('--speed', type=float, default=1.0, metavar='MULTIPLIER',
        help='Playback speed multiplier (default: 1.0). Ignored with --instant.')
    parser.add_argument('--instant', action='store_true',
        help='Send all breadcrumbs immediately with no timing delays.')
    parser.add_argument('--track_speed', type=float, default=0.0, metavar='M_PER_S',
        help='CoT <track speed="..."> fallback in m/s used when a breadcrumb has '
             'no recorded GPS speed (default: 0.0).  Most breadcrumbs use the speed '
             'stored in the database; this only applies to early crumbs recorded '
             'before the GPS speed topic had published.')
    parser.add_argument('--skip_home', action='store_true',
        help='Do not send the home position CoT marker before replaying breadcrumbs.')

    args = parser.parse_args()

    # ── VALIDATE ──────────────────────────────────────────────────────────────

    db_path = Path(args.db_path)
    if not db_path.exists():
        print(f'ERROR: Database file not found: {db_path}', file=sys.stderr)
        sys.exit(1)
    if args.speed <= 0.0:
        print(f'ERROR: --speed must be > 0 (got {args.speed})', file=sys.stderr)
        sys.exit(1)

    robot_name = args.robot_name or infer_robot_name(db_path)

    # ── LOAD ──────────────────────────────────────────────────────────────────

    print(f'\nDB TAK Replay')
    print(f'  Database    : {db_path}')
    print(f'  Robot       : {robot_name}')
    print(f'  Mode        : {"INSTANT" if args.instant else f"{args.speed}x real-time"}')
    print()

    print('Loading breadcrumbs...')
    breadcrumbs = load_breadcrumbs(db_path)

    print('Loading waypoints...')
    waypoints = load_waypoints(db_path)
    if not breadcrumbs:
        print('No valid breadcrumbs found. Nothing to replay.')
        sys.exit(0)

    first_ts = breadcrumbs[0]['ts']
    last_ts  = breadcrumbs[-1]['ts']
    duration = (last_ts - first_ts).total_seconds()

    print(f'  {len(breadcrumbs)} breadcrumbs loaded')
    print(f'  {len(waypoints)} waypoint(s) loaded  '
          f'({sum(1 for w in waypoints if w["status"]=="reached")} reached, '
          f'{sum(1 for w in waypoints if w["status"]=="failed")} failed, '
          f'{sum(1 for w in waypoints if w["status"]=="pending")} pending)')
    print(f'  Mission start : {first_ts.strftime("%Y-%m-%d %H:%M:%S")} UTC')
    print(f'  Mission end   : {last_ts.strftime("%Y-%m-%d %H:%M:%S")} UTC')
    print(f'  Duration      : {duration:.0f}s  ({duration / 60:.1f} min)')
    if not args.instant:
        replay_secs = duration / args.speed
        print(f'  Replay time   : ~{replay_secs:.0f}s  ({replay_secs / 60:.1f} min) at {args.speed}x')
    print()

    # ── KEYBOARD CONTROLLER ───────────────────────────────────────────────────

    controller = KeyboardController()

    if not args.instant:
        tty_ok = controller.start()
        if tty_ok:
            print('Controls: Space/p = pause/resume   q = quit')
        else:
            print('(Stdin is not a TTY -- pause/resume controls unavailable)')
        print()
    else:
        # Start controller anyway so quit-detection via Ctrl+C still works.
        controller.start()

    # ── ROS2 INIT ─────────────────────────────────────────────────────────────

    rclpy.init()
    node = ReplayNode(robot_name)

    # ── WAIT FOR SUBSCRIBER ───────────────────────────────────────────────────
    # Poll until tak_chat (or any subscriber) connects to the send_to_tak
    # topic, with a 10-second timeout.
    #
    # WHY NOT just spin N times:
    #   The original fixed-count spin (10 × 0.1s = 1s) doesn't verify that
    #   anyone actually connected.  If tak_chat is slow to start, all early
    #   messages get silently dropped.
    #
    # WHY the 0.5s stabilization pause after connection is detected:
    #   DDS discovery reports get_subscription_count() > 0 as soon as the
    #   subscriber endpoint is announced, but the transport layer (shared
    #   memory or UDP) finishes handshaking a moment later.  Publishing
    #   immediately after the count goes non-zero still drops the first
    #   message on virtually every DDS implementation.  The 0.5s pause is
    #   the simplest reliable fix.
    print('Waiting for subscriber connection...')
    _sub_deadline = time.monotonic() + 10.0
    while time.monotonic() < _sub_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node._pub.get_subscription_count() > 0:
            break
    else:
        print('WARNING: No subscriber detected after 10s -- messages may be dropped.',
              file=sys.stderr)

    # Stabilization pause -- let DDS transport finish handshaking before we
    # send anything.  Without this the first 1-2 messages are often dropped
    # even though get_subscription_count() > 0.
    time.sleep(0.5)
    print('Ready.\n')

    # ── HOME COT ──────────────────────────────────────────────────────────────

    sent_items: list[dict] = []
    home: dict | None = None

    try:
        if not args.skip_home:
            home = load_home_position(db_path)
            if home is not None:
                print(f'Sending home position CoT: ({home["lat"]:.7f}, {home["lon"]:.7f})'
                      f'  set_by={home["set_by"]}')
                home_xml = build_home_cot_message(robot_name, home['lat'], home['lon'])
                publish_with_retry(node, home_xml)
                print(f'  Callsign : {robot_name.upper()}_HOME')
                # Spin briefly so DDS fully dispatches the home CoT retries
                # before the replay starts sending breadcrumbs.  Without this,
                # the first breadcrumb can collide with in-flight home retries
                # and get dropped.
                rclpy.spin_once(node, timeout_sec=0.2)
                print()
            else:
                print('No home position stored in database -- skipping home CoT.\n')
        else:
            print('Home CoT skipped (--skip_home).\n')

    # ── REPLAY ────────────────────────────────────────────────────────────────

        n_dispatch   = sum(1 for w in waypoints if w['dispatched_ts'] is not None)
        n_completion = sum(1 for w in waypoints if w['status'] in ('reached', 'failed'))
        n_wp_events  = n_dispatch + n_completion
        wp_suffix    = f', {n_wp_events} waypoint event(s)' if n_wp_events else ''

        # Show whether comms data was found in the DB schema
        has_comms = any(c.get('comms_ok') is not None for c in breadcrumbs)
        comms_note = '  (comms status loaded)' if has_comms else '  (no comms data in DB)'
        print(f'Starting replay -- {len(breadcrumbs)} breadcrumb(s){wp_suffix}.{comms_note}\n')
        sent_items = run_replay(
            node             = node,
            breadcrumbs      = breadcrumbs,
            waypoints        = waypoints,
            robot_name       = robot_name,
            track_speed      = args.track_speed,
            speed_multiplier = args.speed,
            instant          = args.instant,
            controller       = controller,
        )

        bc  = getattr(node, '_breadcrumb_count', len(sent_items))
        wpe = getattr(node, '_wp_event_count', 0)
        wp_str = f', {wpe} waypoint event(s)' if wpe else ''
        if not controller.is_quit_requested():
            print(f'\nReplay complete.  {bc} breadcrumb(s){wp_str} sent.')
        else:
            print(f'\nReplay stopped.  {bc} breadcrumb(s){wp_str} sent before quit.')

    except KeyboardInterrupt:
        controller.request_quit()
        print(f'\nInterrupted.  {len(sent_items)} breadcrumb(s) sent before interrupt.')

    finally:
        # Always restore the terminal before input() -- runs regardless of how
        # the replay ended (normal, q-quit, Ctrl+C, or any other exception).
        controller.stop()

    # ── DELETE PROMPT ──────────────────────────────────────────────────────────
    # Runs after the try/finally so it is reached via any exit path:
    # normal completion, q-quit, or Ctrl+C.  The node is still alive here so
    # delete CoTs can be published before rclpy.shutdown().

    if sent_items:
        home_uid = (
            str(uuid.uuid5(uuid.NAMESPACE_URL,
                           f'mission_database/{robot_name}/home'))
            if (home is not None and not args.skip_home)
            else None
        )

        wp_uids = [
            (wp['waypoint_id'],
             str(uuid.uuid5(uuid.NAMESPACE_URL,
                            f'mission_database/{robot_name}/waypoint/{wp["waypoint_id"]}')
             ),
             wp['lat'], wp['lon'])
            for wp in waypoints
        ] if waypoints and not args.skip_home else []

        total_symbols = (len(sent_items)
                         + (1 if home_uid else 0)
                         + len(wp_uids))

        parts = [f'{len(sent_items)} breadcrumbs']
        if home_uid:  parts.append('1 home marker')
        if wp_uids:   parts.append(f'{len(wp_uids)} waypoint circle(s)')

        print(f'\nDelete {total_symbols} symbol(s) from ATAK? '
              f'({", ".join(parts)}) [y/N] ', end='', flush=True)

        try:
            answer = input().strip().lower()
        except EOFError:
            answer = 'n'

        if answer == 'y':
            print(f'Sending {total_symbols} delete CoT(s)...')

            for item in sent_items:
                del_xml = build_delete_cot_message(
                    item['uid'], item['lat'], item['lon']
                )
                publish_with_retry(node, del_xml)

            if home_uid and home is not None:
                del_xml = build_delete_cot_message(
                    home_uid, home['lat'], home['lon']
                )
                publish_with_retry(node, del_xml)

            for wp_id, wp_uid, wp_lat, wp_lon in wp_uids:
                del_xml = build_delete_cot_message(wp_uid, wp_lat, wp_lon)
                publish_with_retry(node, del_xml)

            rclpy.spin_once(node, timeout_sec=0.1)
            print('Delete commands sent.')
        else:
            print('Symbols kept on ATAK.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
