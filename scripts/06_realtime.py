#!/usr/bin/env python3
"""Control each joint in real time by commanding absolute target angles.

Phases:
  waiting — torque stays off until Enter
  prepare — drive the arm to its home pose and wait until it is there
  running — keyboard changes one joint's target; servos are commanded to it
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import arm_settings as settings
from arm_helpers import (
    angle_to_position,
    connect,
    disable_all_torque,
    enable_all_torque,
    move_all_sync,
    position_to_angle,
    read_all,
    wait_for_servos_or_estop,
)
from terminal_keys import RawTerminal

# One keypress (or one key-repeat) changes that joint's target by this many degrees.
STEP_DEGREES = 3.0
# Prepare finishes when every servo is within this many ticks of its home
# angle (about 3.5 degrees). This is a completion check, not a control gain.
SETTLE_TICKS = 40
PREPARE_TIMEOUT_SECONDS = 30.0
HOME_DEGREES = 0.0
GRIPPER_HOME_DEGREES = 41.22

# Degrees in joint order: base, shoulder, elbow, wrist, gripper.
# K plays these with the same speed, acceleration, and Space stop as waypoint playback.
KEY_WAYPOINTS = (
    (-1.41, -4.92, 8.35, -92.55, 27.86),
    (-1.49, -48.78, -17.67, -47.72, 6.50),
    (-2.72, -38.06, 7.29, -63.63, 6.50),
    (-2.64, -75.23, 41.92, -61.79, 6.50),
    (-2.64, -75.23, 41.92, -61.79, 22.41),
    (-2.11, 9.05, -17.67, -70.40, 22.41),
    (-1.41, -4.92, 8.35, -92.55, 27.86),
)

# Side view of the 0, 0, 0 pose. +Y is up. +X points to the right of the
# sketch, which is the side the wrist sits on (forward). Link vectors were
# measured between the shoulder, elbow, and wrist circles. Positive shoulder,
# elbow, and wrist angles rotate counter-clockwise; s, d, and f decrease those
# angles and move the joints clockwise.
UPPER_ARM = (-144.0, 192.0)
FOREARM = (280.0, 320.0)
GRIPPER = (5.5, 60.0)
LINK_LENGTHS = tuple(math.hypot(x, y) for x, y in (UPPER_ARM, FOREARM, GRIPPER))
ZERO_LINK_ANGLES = tuple(math.atan2(y, x) for x, y in (UPPER_ARM, FOREARM, GRIPPER))
# One keypress moves the gripper by this much, in the same units as the links.
TRANSLATION_STEP = 36.0
TRANSLATION_KEYS = {
    "v": (0.0, TRANSLATION_STEP),
    "b": (0.0, -TRANSLATION_STEP),
    "n": (TRANSLATION_STEP, 0.0),
    "m": (-TRANSLATION_STEP, 0.0),
}

# First key increases the calibrated angle, second key decreases it.
JOINT_KEYS = {
    "base_yaw": ("q", "a"),
    "shoulder": ("w", "s"),
    "elbow": ("e", "d"),
    "wrist": ("r", "f"),
    "gripper": ("t", "g"),
}


def key_steps():
    """Map a single key to (joint name, degree change)."""
    steps = {}
    for name, (increase_key, decrease_key) in JOINT_KEYS.items():
        steps[increase_key] = (name, STEP_DEGREES)
        steps[decrease_key] = (name, -STEP_DEGREES)
    return steps


def home_angles():
    angles = {name: HOME_DEGREES for name in settings.JOINT_NAMES}
    angles["gripper"] = GRIPPER_HOME_DEGREES
    return angles


def angles_to_positions(angles):
    return {
        name: angle_to_position(name, angles[name]) for name in settings.JOINT_NAMES
    }


def format_angles(angles):
    return "  ".join(
        f"{name} {angles[name]:+6.1f}" for name in settings.JOINT_NAMES
    )


def _wrap_degrees(angle):
    return (angle + 180.0) % 360.0 - 180.0


def _within_limits(name, angle):
    minimum, maximum = settings.ANGLE_LIMITS[name]
    return minimum <= angle <= maximum


def tool_pose(shoulder, elbow, wrist):
    """Gripper position and tilt for software joint angles, in degrees."""
    shoulder_rad = math.radians(shoulder)
    elbow_rad = math.radians(elbow)
    wrist_rad = math.radians(wrist)
    absolute = (
        ZERO_LINK_ANGLES[0] + shoulder_rad,
        ZERO_LINK_ANGLES[1] + shoulder_rad + elbow_rad,
        ZERO_LINK_ANGLES[2] + shoulder_rad + elbow_rad + wrist_rad,
    )
    x = 0.0
    y = 0.0
    for length, angle in zip(LINK_LENGTHS, absolute):
        x += length * math.cos(angle)
        y += length * math.sin(angle)
    return x, y, absolute[2]


def move_tool(angles, dx, dy):
    """Move the gripper in the arm plane without changing its tilt.

    Shoulder, elbow, and wrist are solved together. Base and gripper stay put.
    Returns False when the step would leave the reachable area or a joint limit.
    """
    x, y, pitch = tool_pose(angles["shoulder"], angles["elbow"], angles["wrist"])
    upper, forearm, gripper = LINK_LENGTHS
    wrist_x = (x + dx) - gripper * math.cos(pitch)
    wrist_y = (y + dy) - gripper * math.sin(pitch)
    reach_sq = wrist_x * wrist_x + wrist_y * wrist_y
    cosine = (reach_sq - upper * upper - forearm * forearm) / (2.0 * upper * forearm)
    if cosine < -1.0 or cosine > 1.0:
        return False

    current_elbow = (ZERO_LINK_ANGLES[1] - ZERO_LINK_ANGLES[0]) + math.radians(
        angles["elbow"]
    )
    bent = math.acos(max(-1.0, min(1.0, cosine)))
    elbow_bend = bent if abs(bent - current_elbow) <= abs(-bent - current_elbow) else -bent
    shoulder_angle = math.atan2(wrist_y, wrist_x) - math.atan2(
        forearm * math.sin(elbow_bend),
        upper + forearm * math.cos(elbow_bend),
    )
    shoulder = _wrap_degrees(math.degrees(shoulder_angle - ZERO_LINK_ANGLES[0]))
    elbow = _wrap_degrees(
        math.degrees(elbow_bend - (ZERO_LINK_ANGLES[1] - ZERO_LINK_ANGLES[0]))
    )
    wrist = _wrap_degrees(math.degrees(pitch - ZERO_LINK_ANGLES[2]) - shoulder - elbow)
    if not (
        _within_limits("shoulder", shoulder)
        and _within_limits("elbow", elbow)
        and _within_limits("wrist", wrist)
    ):
        return False
    angles["shoulder"] = shoulder
    angles["elbow"] = elbow
    angles["wrist"] = wrist
    return True


def apply_translation(angles, dx, dy):
    """Try the full step, then a shorter one, so a limit does not freeze the key."""
    saved = {name: angles[name] for name in ("shoulder", "elbow", "wrist")}
    for scale in (1.0, 0.5, 0.25):
        for name, angle in saved.items():
            angles[name] = angle
        if move_tool(angles, dx * scale, dy * scale):
            return True
    for name, angle in saved.items():
        angles[name] = angle
    return False


def apply_key(angles, key, steps):
    """Shift one joint target by one step, staying inside ANGLE_LIMITS."""
    binding = steps.get(key)
    if binding is None:
        return False
    name, delta = binding
    minimum, maximum = settings.ANGLE_LIMITS[name]
    updated = min(maximum, max(minimum, angles[name] + delta))
    if updated == angles[name]:
        return False
    angles[name] = updated
    return True


def read_angles(packet_handler):
    positions = read_all(packet_handler)
    return {
        name: position_to_angle(name, positions[name]) for name in settings.JOINT_NAMES
    }, positions


def near_targets(positions, targets):
    return all(
        abs(positions[name] - targets[name]) <= SETTLE_TICKS
        for name in settings.JOINT_NAMES
    )


def command_positions(packet_handler, positions):
    """Command raw ticks. The servo position controller drives to that goal."""
    move_all_sync(
        packet_handler,
        positions,
        speed=settings.MOVE_SPEED,
        acceleration=settings.MOVE_ACCELERATION,
    )


def command_angles(packet_handler, angles):
    command_positions(packet_handler, angles_to_positions(angles))


def waypoint_positions():
    """Convert the K sequence from degrees into raw servo goals."""
    positions = []
    for point in KEY_WAYPOINTS:
        positions.append(
            {
                name: angle_to_position(name, angle)
                for name, angle in zip(settings.JOINT_NAMES, point)
            }
        )
    return positions


def play_key_waypoints(packet_handler, terminal, angles):
    """Play the K sequence the same way a saved waypoint file is played."""
    points = waypoint_positions()
    servo_ids = [settings.SERVO_IDS[name] for name in settings.JOINT_NAMES]
    print("\r\nPlaying K waypoint sequence. Press Space for E-stop.", flush=True)
    for index, positions in enumerate(points, start=1):
        if terminal.read_key(0) == " ":
            print("\r\nEMERGENCY STOP: releasing torque on all servos.", flush=True)
            disable_all_torque(packet_handler)
            return False
        move_all_sync(
            packet_handler,
            positions,
            speed=settings.POSE_SEQUENCE_MOVE_SPEED,
            acceleration=settings.POSE_SEQUENCE_MOVE_ACCELERATION,
        )
        print(
            f"\rMoving to point {index}/{len(points)} (Space = E-stop)",
            end="",
            flush=True,
        )
        if not wait_for_servos_or_estop(packet_handler, servo_ids, terminal):
            return False
    print("\r\nWaypoint sequence finished.", flush=True)
    for name, angle in zip(settings.JOINT_NAMES, KEY_WAYPOINTS[-1]):
        angles[name] = angle
    while terminal.read_key(0) is not None:
        pass
    return True


def prepare_home(packet_handler, angles):
    """Turn torque on, then drive every joint to home the way waypoint playback does."""
    enable_all_torque(packet_handler)
    targets = angles_to_positions(angles)
    servo_ids = [settings.SERVO_IDS[name] for name in settings.JOINT_NAMES]
    deadline = time.monotonic() + PREPARE_TIMEOUT_SECONDS
    with RawTerminal() as terminal:
        while True:
            if terminal.read_key(0) in (" ", "\x03"):
                print("\r\nEMERGENCY STOP: releasing torque on all servos.", flush=True)
                disable_all_torque(packet_handler)
                return False
            command_angles(packet_handler, angles)
            if not wait_for_servos_or_estop(packet_handler, servo_ids, terminal):
                return False
            positions = read_all(packet_handler)
            if near_targets(positions, targets):
                return True
            if time.monotonic() >= deadline:
                measured = {
                    name: position_to_angle(name, positions[name])
                    for name in settings.JOINT_NAMES
                }
                raise RuntimeError(
                    "Prepare timed out before the arm reached its home pose. "
                    f"Measured: {format_angles(measured)}"
                )
            time.sleep(0.1)


def print_instructions():
    print("Realtime joint control — five motors, absolute angles")
    print("Order: " + " → ".join(settings.JOINT_NAMES))
    print()
    print("Phases")
    print("  waiting   no torque, until you press Enter")
    print("  prepare   move to 0 0 0 0 41.22 degrees and wait until settled")
    print("  running   each key sets one joint; the servo is driven to that angle")
    print()
    print(f"Running keys (each step is {STEP_DEGREES:g} degree, held keys repeat):")
    for name in settings.JOINT_NAMES:
        increase_key, decrease_key = JOINT_KEYS[name]
        minimum, maximum = settings.ANGLE_LIMITS[name]
        print(
            f"  {increase_key}/{decrease_key}  {name:12s} "
            f"+/-   limits {minimum:g} to {maximum:g} deg"
        )
    print("  v/b    gripper up / down")
    print("  n/m    gripper forward / back")
    print("         v, b, n, and m move shoulder, elbow, and wrist together")
    print("  k      play the bound waypoint sequence")
    print("  space  emergency stop (torque off)")
    print("  x      quit (torque off)")
    print()
    print("Positive angles follow DIRECTIONS in arm_settings.py.")
    print("Support the arm while torque is off.")
    print()


def run_keyboard(packet_handler, angles):
    """Running phase: update one target per key and command that pose."""
    steps = key_steps()
    print("RUNNING — keyboard control is live.", flush=True)
    print(f"  {format_angles(angles)}", flush=True)
    with RawTerminal() as terminal:
        while True:
            key = terminal.read_key(0.05)
            if key is None:
                continue
            if key == " " or key == "\x03":
                print("\r\nEMERGENCY STOP: releasing torque on all servos.", flush=True)
                disable_all_torque(packet_handler)
                return
            if key == "x":
                print("\r\nQuit.", flush=True)
                return
            if key == "k":
                if not play_key_waypoints(packet_handler, terminal, angles):
                    return
                sys.stdout.write("\r" + format_angles(angles) + "\x1b[K")
                sys.stdout.flush()
                continue
            if key in TRANSLATION_KEYS:
                dx, dy = TRANSLATION_KEYS[key]
                if not apply_translation(angles, dx, dy):
                    continue
                command_angles(packet_handler, angles)
                sys.stdout.write("\r" + format_angles(angles) + "\x1b[K")
                sys.stdout.flush()
                continue
            if not apply_key(angles, key, steps):
                continue
            command_angles(packet_handler, angles)
            sys.stdout.write("\r" + format_angles(angles) + "\x1b[K")
            sys.stdout.flush()


def main():
    print_instructions()
    print(f"Opening {settings.SERIAL_PORT} at {settings.BAUD_RATE} baud...")
    port = None
    packet_handler = None
    try:
        port, packet_handler = connect()
        disable_all_torque(packet_handler)
        measured, _positions = read_angles(packet_handler)
        print("WAITING — torque is off.")
        print(f"  measured: {format_angles(measured)}")
        input("Press Enter to start PREPARE (return to 0 0 0 0 41.22)... ")

        print("PREPARE — moving to 0 0 0 0 41.22 degrees. Press Space for E-stop.")
        angles = home_angles()
        if not prepare_home(packet_handler, angles):
            return
        settled, _positions = read_angles(packet_handler)
        print(f"Settled near home. Measured: {format_angles(settled)}")
        run_keyboard(packet_handler, angles)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if packet_handler is not None:
            try:
                disable_all_torque(packet_handler)
                print("Torque released. Support the arm.")
            except RuntimeError as error:
                print(f"Could not release torque: {error}", file=sys.stderr)
        if port is not None:
            port.closePort()
            print("Port closed.")


if __name__ == "__main__":
    main()
