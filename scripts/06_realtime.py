#!/usr/bin/env python3
"""Control each joint in real time by commanding absolute target angles.

Phases:
  waiting — torque stays off until Enter
  prepare — drive every joint to 0 degrees and wait until it is there
  running — keyboard changes one joint's target; servos are commanded to it
"""

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
# Prepare finishes when every servo is within this many ticks of 0 degrees
# (about 3.5 degrees). This is a completion check, not a control gain.
SETTLE_TICKS = 40
PREPARE_TIMEOUT_SECONDS = 30.0
HOME_DEGREES = 0.0

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
    return {name: HOME_DEGREES for name in settings.JOINT_NAMES}


def angles_to_positions(angles):
    return {
        name: angle_to_position(name, angles[name]) for name in settings.JOINT_NAMES
    }


def format_angles(angles):
    return "  ".join(
        f"{name} {angles[name]:+6.1f}" for name in settings.JOINT_NAMES
    )


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
                    "Prepare timed out before the arm reached 0 degrees. "
                    f"Measured: {format_angles(measured)}"
                )
            time.sleep(0.1)


def print_instructions():
    print("Realtime joint control — five motors, absolute angles")
    print("Order: " + " → ".join(settings.JOINT_NAMES))
    print()
    print("Phases")
    print("  waiting   no torque, until you press Enter")
    print("  prepare   move to 0 0 0 0 0 degrees and wait until settled")
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
        input("Press Enter to start PREPARE (return to 0 0 0 0 0)... ")

        print("PREPARE — moving to 0 0 0 0 0 degrees. Press Space for E-stop.")
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
