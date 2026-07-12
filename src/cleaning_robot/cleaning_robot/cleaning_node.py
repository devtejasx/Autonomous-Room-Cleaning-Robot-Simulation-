import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class CleaningRobot(Node):
    def __init__(self):
        super().__init__('cleaning_robot')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage_trail', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.timer = self.create_timer(0.1, self.move)

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False

        # LiDAR / obstacle detection
        self.obstacle_front = False
        self.front_min = 10.0
        self.obstacle_threshold_front = 0.55
        self.shift_emergency_threshold = 0.28

        # Cleaning area visualization
        self.clean_patch_size = 0.40
        self.clean_spacing = 0.18
        self.cleaned_cells = set()
        self.markers = MarkerArray()
        self.marker_id = 0
        self.last_clean_x = None
        self.last_clean_y = None

        # Start corner
        self.phase = 'GOTO_CORNER'
        self.corner_x = -3.2
        self.corner_y = -3.2

        # Zig-zag
        self.state = 'IDLE'
        self.target_yaw = 0.0

        self.row_start_x = 0.0
        self.row_start_y = 0.0
        self.shift_start_x = 0.0
        self.shift_start_y = 0.0

        self.row = 0
        self.max_rows = 14
        self.row_length = 6.2
        self.row_spacing = 0.45
        self.min_row_progress_before_end = 0.45

        # 1 = left-left, -1 = right-right
        self.turn_dir = 1

        # Bypass state
        self.bypass_start_x = 0.0
        self.bypass_start_y = 0.0
        self.bypass_target_yaw = 0.0
        self.bypass_forward_start_x = 0.0
        self.bypass_forward_start_y = 0.0
        self.bypass_forward_distance = 0.9
        self.bypass_side_sign = 1
        self.saved_row_heading = 0.0

        self.get_logger().info('🤖 Going to starting corner...')

    # ---------------- callbacks ----------------

    def odom_callback(self, msg: Odometry):
        self.have_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        if self.phase == 'CLEAN':
            self.update_cleaned_area()

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        front = msg.ranges[0:25] + msg.ranges[-25:]
        valid = [r for r in front if 0.05 < r < 10.0]

        if valid:
            self.front_min = min(valid)
            self.obstacle_front = self.front_min < self.obstacle_threshold_front
        else:
            self.front_min = 10.0
            self.obstacle_front = False

    # ---------------- helpers ----------------

    def angle_diff(self, target, current):
        d = target - current
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def dist_to(self, tx, ty):
        return math.sqrt((self.x - tx) ** 2 + (self.y - ty) ** 2)

    def face_towards(self, tx, ty):
        return math.atan2(ty - self.y, tx - self.x)

    def dist_from_row_start(self):
        return math.sqrt(
            (self.x - self.row_start_x) ** 2 +
            (self.y - self.row_start_y) ** 2
        )

    def dist_from_shift_start(self):
        return math.sqrt(
            (self.x - self.shift_start_x) ** 2 +
            (self.y - self.shift_start_y) ** 2
        )

    def dist_from_bypass_start(self):
        return math.sqrt(
            (self.x - self.bypass_start_x) ** 2 +
            (self.y - self.bypass_start_y) ** 2
        )

    def dist_from_bypass_forward_start(self):
        return math.sqrt(
            (self.x - self.bypass_forward_start_x) ** 2 +
            (self.y - self.bypass_forward_start_y) ** 2
        )

    def current_row_heading(self):
        return 0.0 if self.turn_dir == 1 else math.pi

    # ---------------- cleaned area ----------------

    def update_cleaned_area(self):
        if self.last_clean_x is None:
            self.last_clean_x = self.x
            self.last_clean_y = self.y
            self.add_clean_patch(self.x, self.y)
            return

        dx = self.x - self.last_clean_x
        dy = self.y - self.last_clean_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.clean_spacing:
            return

        self.last_clean_x = self.x
        self.last_clean_y = self.y

        cell_x = round(self.x / (self.clean_patch_size * 0.5))
        cell_y = round(self.y / (self.clean_patch_size * 0.5))
        cell = (cell_x, cell_y)

        if cell not in self.cleaned_cells:
            self.cleaned_cells.add(cell)
            self.add_clean_patch(self.x, self.y)

    def add_clean_patch(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'cleaned_area'
        marker.id = self.marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.005
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.clean_patch_size
        marker.scale.y = self.clean_patch_size
        marker.scale.z = 0.01

        marker.color.r = 0.1
        marker.color.g = 0.85
        marker.color.b = 0.2
        marker.color.a = 0.45
        marker.lifetime.sec = 0

        self.markers.markers.append(marker)
        self.marker_pub.publish(self.markers)
        self.marker_id += 1

    # ---------------- motion ----------------

    def move(self):
        if not self.have_odom:
            return

        msg = Twist()

        # -------- Go to start corner --------
        if self.phase == 'GOTO_CORNER':
            target_angle = self.face_towards(self.corner_x, self.corner_y)
            diff = self.angle_diff(target_angle, self.yaw)
            dist = self.dist_to(self.corner_x, self.corner_y)

            if dist < 0.35:
                self.phase = 'CLEAN'
                self.state = 'ALIGN'
                self.target_yaw = 0.0
                self.get_logger().info('✅ At corner! Starting vacuum coverage...')
            elif abs(diff) > 0.12:
                msg.angular.z = 1.0 if diff > 0 else -1.0
            else:
                msg.linear.x = 0.28
                msg.angular.z = 0.30 * diff

            self.cmd_pub.publish(msg)
            return

        # -------- Zig-zag cleaning --------

        if self.state == 'ALIGN':
            diff = self.angle_diff(self.target_yaw, self.yaw)
            if abs(diff) > 0.05:
                msg.angular.z = 0.9 if diff > 0 else -0.9
            else:
                self.state = 'FORWARD'
                self.row_start_x = self.x
                self.row_start_y = self.y
                self.get_logger().info(f'➡️ Row {self.row} started')

        elif self.state == 'FORWARD':
            row_progress = self.dist_from_row_start()

            hit_row_end = row_progress >= self.row_length
            hit_obstacle = self.obstacle_front and row_progress > 0.30

            if hit_obstacle:
                self.get_logger().info('⚠️ Obstacle detected → bypassing')

                self.saved_row_heading = self.current_row_heading()

                # Pick bypass side from current sweep direction:
                # moving east (turn_dir=1) -> go upward first
                # moving west (turn_dir=-1) -> go downward first
                self.bypass_side_sign = 1 if self.turn_dir == 1 else -1

                self.state = 'BYPASS_TURN1'
                self.bypass_target_yaw = self.normalize_angle(
                    self.saved_row_heading + self.bypass_side_sign * math.pi / 2.0
                )

                self.bypass_start_x = self.x
                self.bypass_start_y = self.y

            elif hit_row_end:
                self.get_logger().info(f'✅ Row {self.row} complete')
                self.state = 'TURN1'
                self.target_yaw = self.normalize_angle(
                    self.yaw + self.turn_dir * math.pi / 2.0
                )

            else:
                msg.linear.x = 0.24
                msg.angular.z = 0.0

        elif self.state == 'BYPASS_TURN1':
            diff = self.angle_diff(self.bypass_target_yaw, self.yaw)
            if abs(diff) > 0.05:
                msg.angular.z = 0.9 if diff > 0 else -0.9
            else:
                self.state = 'BYPASS_SHIFT'
                self.bypass_start_x = self.x
                self.bypass_start_y = self.y

        elif self.state == 'BYPASS_SHIFT':
            dist = self.dist_from_bypass_start()

            if dist >= self.row_spacing * 1.6:
                self.state = 'BYPASS_TURN2'
                self.bypass_target_yaw = self.saved_row_heading
            elif self.front_min < self.shift_emergency_threshold:
                # Something very close while shifting, stop shift early and
                # try to rejoin the row heading anyway
                self.state = 'BYPASS_TURN2'
                self.bypass_target_yaw = self.saved_row_heading
            else:
                msg.linear.x = 0.16
                msg.angular.z = 0.0

        elif self.state == 'BYPASS_TURN2':
            diff = self.angle_diff(self.bypass_target_yaw, self.yaw)
            if abs(diff) > 0.05:
                msg.angular.z = 0.9 if diff > 0 else -0.9
            else:
                self.state = 'BYPASS_FORWARD'
                self.bypass_forward_start_x = self.x
                self.bypass_forward_start_y = self.y

        elif self.state == 'BYPASS_FORWARD':
            forward_dist = self.dist_from_bypass_forward_start()

            # Continue along the same row heading until we have clearly passed
            # the obstacle footprint. Then resume normal row cleaning.
            if (not self.obstacle_front and forward_dist >= self.bypass_forward_distance) or \
               (forward_dist >= self.bypass_forward_distance * 1.5):
                self.get_logger().info('✅ Passed obstacle, resuming same row')
                self.state = 'FORWARD'
            else:
                msg.linear.x = 0.20
                msg.angular.z = 0.0

        elif self.state == 'TURN1':
            diff = self.angle_diff(self.target_yaw, self.yaw)
            if abs(diff) > 0.05:
                msg.angular.z = 0.9 if diff > 0 else -0.9
            else:
                self.state = 'SHIFT'
                self.shift_start_x = self.x
                self.shift_start_y = self.y

        elif self.state == 'SHIFT':
            shift_progress = self.dist_from_shift_start()

            # During shift, ignore the normal row obstacle threshold.
            # Only stop shift early if something is VERY close.
            if shift_progress >= self.row_spacing:
                self.state = 'TURN2'
                self.target_yaw = self.normalize_angle(
                    self.yaw + self.turn_dir * math.pi / 2.0
                )
            elif self.front_min < self.shift_emergency_threshold:
                self.state = 'TURN2'
                self.target_yaw = self.normalize_angle(
                    self.yaw + self.turn_dir * math.pi / 2.0
                )
            else:
                msg.linear.x = 0.16
                msg.angular.z = 0.0

        elif self.state == 'TURN2':
            diff = self.angle_diff(self.target_yaw, self.yaw)
            if abs(diff) > 0.05:
                msg.angular.z = 0.9 if diff > 0 else -0.9
            else:
                self.row += 1
                self.turn_dir *= -1
                self.state = 'FORWARD'
                self.row_start_x = self.x
                self.row_start_y = self.y

                if self.row >= self.max_rows:
                    self.state = 'DONE'
                    self.get_logger().info(
                        f'🎉 ROOM CLEANED. Total cleaned patches: {len(self.cleaned_cells)}'
                    )
                else:
                    self.get_logger().info(f'➡️ Row {self.row} started')

        elif self.state == 'DONE':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('🛑 Cleaning complete. Robot stopped.', once=True)

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CleaningRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
