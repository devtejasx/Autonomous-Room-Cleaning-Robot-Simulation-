"""Unit tests for the CleaningRobot zig-zag/bypass state machine.

The ROS 2 modules are stubbed before importing the node, so these tests run
hermetically anywhere (pytest or plain `python test_state_machine.py`) —
no rclpy installation, ROS graph, or Gazebo needed.
"""

import math
import sys
import types


# --------------------------------------------------------------------------
# Minimal ROS 2 stubs (installed unconditionally so the tests are hermetic)
# --------------------------------------------------------------------------

class _AttrBag:
    """Object that grows attribute bags on demand (message-like)."""

    def __getattr__(self, name):
        value = _AttrBag()
        object.__setattr__(self, name, value)
        return value


class _Logger:
    def info(self, *args, **kwargs):
        pass


class _Clock:
    def now(self):
        return self

    def to_msg(self):
        return None


class _Publisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _Node:
    def __init__(self, name):
        self._publishers = []

    def create_publisher(self, msg_type, topic, qos):
        pub = _Publisher()
        self._publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        return None

    def create_timer(self, period, callback):
        return None

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return _Clock()


class _Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class _Marker(_AttrBag):
    CYLINDER = 3
    ADD = 0


class _MarkerArray:
    def __init__(self):
        self.markers = []


def _module(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    sys.modules[name] = mod
    return mod


_module('rclpy', init=lambda **k: None, spin=lambda n: None, shutdown=lambda: None)
_module('rclpy.node', Node=_Node)
_module('geometry_msgs')
_module('geometry_msgs.msg', Twist=_Twist)
_module('nav_msgs')
_module('nav_msgs.msg', Odometry=object)
_module('sensor_msgs')
_module('sensor_msgs.msg', LaserScan=object)
_module('visualization_msgs')
_module('visualization_msgs.msg', Marker=_Marker, MarkerArray=_MarkerArray)

import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from cleaning_robot.cleaning_node import CleaningRobot  # noqa: E402


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

def make_robot():
    robot = CleaningRobot()
    robot.have_odom = True
    return robot


def make_scan(ranges):
    return types.SimpleNamespace(ranges=list(ranges))


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------

def test_angle_diff_wraps_across_pi():
    robot = make_robot()
    assert abs(robot.angle_diff(math.pi - 0.1, -math.pi + 0.1) - (-0.2)) < 1e-9
    assert abs(robot.angle_diff(0.5, 0.2) - 0.3) < 1e-9


def test_normalize_angle():
    robot = make_robot()
    assert abs(robot.normalize_angle(3 * math.pi) - math.pi) < 1e-9
    assert abs(robot.normalize_angle(-3 * math.pi) + math.pi) < 1e-9


def test_current_row_heading_follows_turn_dir():
    robot = make_robot()
    robot.turn_dir = 1
    assert robot.current_row_heading() == 0.0
    robot.turn_dir = -1
    assert robot.current_row_heading() == math.pi


# --------------------------------------------------------------------------
# Obstacle detection
# --------------------------------------------------------------------------

def test_scan_flags_close_frontal_obstacle():
    robot = make_robot()
    robot.scan_callback(make_scan([0.4] * 30 + [5.0] * 300))
    assert robot.obstacle_front
    assert abs(robot.front_min - 0.4) < 1e-9


def test_scan_ignores_out_of_range_readings():
    robot = make_robot()
    robot.scan_callback(make_scan([0.0] * 30 + [11.0] * 300))
    assert not robot.obstacle_front
    assert robot.front_min == 10.0


# --------------------------------------------------------------------------
# Phase / state transitions
# --------------------------------------------------------------------------

def test_goto_corner_switches_to_clean_at_corner():
    robot = make_robot()
    robot.x, robot.y = robot.corner_x + 0.1, robot.corner_y + 0.1
    robot.move()
    assert robot.phase == 'CLEAN'
    assert robot.state == 'ALIGN'


def test_align_starts_row_when_heading_matches():
    robot = make_robot()
    robot.phase, robot.state = 'CLEAN', 'ALIGN'
    robot.target_yaw = 0.0
    robot.yaw = 0.01
    robot.x, robot.y = 1.0, 2.0
    robot.move()
    assert robot.state == 'FORWARD'
    assert (robot.row_start_x, robot.row_start_y) == (1.0, 2.0)


def test_forward_turns_at_row_end():
    robot = make_robot()
    robot.phase, robot.state = 'CLEAN', 'FORWARD'
    robot.row_start_x, robot.row_start_y = 0.0, 0.0
    robot.x = robot.row_length + 0.01
    robot.move()
    assert robot.state == 'TURN1'


def test_forward_enters_bypass_on_obstacle():
    robot = make_robot()
    robot.phase, robot.state = 'CLEAN', 'FORWARD'
    robot.row_start_x, robot.row_start_y = 0.0, 0.0
    robot.x = 1.0  # past min progress
    robot.obstacle_front = True
    robot.turn_dir = 1
    robot.move()
    assert robot.state == 'BYPASS_TURN1'
    assert abs(robot.bypass_target_yaw - math.pi / 2.0) < 1e-9


def test_obstacle_ignored_at_row_start():
    robot = make_robot()
    robot.phase, robot.state = 'CLEAN', 'FORWARD'
    robot.row_start_x, robot.row_start_y = 0.0, 0.0
    robot.x = 0.1  # within the 0.30 m grace distance
    robot.obstacle_front = True
    robot.move()
    assert robot.state == 'FORWARD'


def test_full_bypass_sequence_rejoins_row():
    robot = make_robot()
    robot.phase = 'CLEAN'
    robot.turn_dir = 1

    # BYPASS_TURN1 -> BYPASS_SHIFT once facing the bypass heading
    robot.state = 'BYPASS_TURN1'
    robot.bypass_target_yaw = math.pi / 2.0
    robot.yaw = math.pi / 2.0
    robot.move()
    assert robot.state == 'BYPASS_SHIFT'

    # BYPASS_SHIFT -> BYPASS_TURN2 after shifting far enough sideways
    robot.saved_row_heading = 0.0
    robot.bypass_start_x, robot.bypass_start_y = robot.x, robot.y
    robot.y += robot.row_spacing * 1.6 + 0.01
    robot.move()
    assert robot.state == 'BYPASS_TURN2'
    assert robot.bypass_target_yaw == 0.0

    # BYPASS_TURN2 -> BYPASS_FORWARD once back on the row heading
    robot.yaw = 0.0
    robot.move()
    assert robot.state == 'BYPASS_FORWARD'

    # BYPASS_FORWARD -> FORWARD after clearing the obstacle footprint
    robot.obstacle_front = False
    robot.bypass_forward_start_x, robot.bypass_forward_start_y = robot.x, robot.y
    robot.x += robot.bypass_forward_distance + 0.01
    robot.move()
    assert robot.state == 'FORWARD'


def test_bypass_shift_aborts_when_blocked():
    robot = make_robot()
    robot.phase, robot.state = 'CLEAN', 'BYPASS_SHIFT'
    robot.saved_row_heading = 0.0
    robot.bypass_start_x, robot.bypass_start_y = robot.x, robot.y
    robot.front_min = robot.shift_emergency_threshold - 0.01
    robot.move()
    assert robot.state == 'BYPASS_TURN2'


def test_turn2_advances_row_and_flips_direction():
    robot = make_robot()
    robot.phase, robot.state = 'CLEAN', 'TURN2'
    robot.target_yaw = 0.0
    robot.yaw = 0.0
    robot.row, robot.turn_dir = 3, 1
    robot.move()
    assert robot.row == 4
    assert robot.turn_dir == -1
    assert robot.state == 'FORWARD'


def test_turn2_finishes_after_max_rows():
    robot = make_robot()
    robot.phase, robot.state = 'CLEAN', 'TURN2'
    robot.target_yaw = 0.0
    robot.yaw = 0.0
    robot.row = robot.max_rows - 1
    robot.move()
    assert robot.state == 'DONE'


def test_done_state_publishes_zero_velocity():
    robot = make_robot()
    robot.phase, robot.state = 'CLEAN', 'DONE'
    robot.move()
    msg = robot.cmd_pub.published[-1]
    assert msg.linear.x == 0.0
    assert msg.angular.z == 0.0


# --------------------------------------------------------------------------
# Coverage tracking
# --------------------------------------------------------------------------

def test_cleaned_cells_deduplicate():
    robot = make_robot()
    robot.phase = 'CLEAN'
    robot.x, robot.y = 0.0, 0.0
    robot.update_cleaned_area()          # first patch at origin
    robot.x = robot.clean_spacing + 0.01  # move one spacing away
    robot.update_cleaned_area()
    cells_after_two = len(robot.cleaned_cells)
    robot.update_cleaned_area()           # same spot: no new cell
    assert len(robot.cleaned_cells) == cells_after_two


if __name__ == '__main__':
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f'PASS {name}')
            except AssertionError as exc:
                failures += 1
                print(f'FAIL {name}: {exc}')
    print(f'\n{failures} failures')
    raise SystemExit(1 if failures else 0)
