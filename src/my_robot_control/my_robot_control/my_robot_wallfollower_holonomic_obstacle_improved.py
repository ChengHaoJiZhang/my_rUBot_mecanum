#!/usr/bin/env python3
"""Student template for the holonomic obstacle wall follower."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion


def clamp(value, low, high):
    return max(low, min(high, value))


class HolonomicObstacleWallFollower(Node):
    """Complete the LiDAR processing, state machine and control laws."""

    def __init__(self):
        super().__init__('holonomic_obstacle_wall_follower')

        # Desired distance between the robot and the right wall (m).
        self.follow_distance = float(
            self.declare_parameter('follow_distance', 0.35).value
        )
        # Maximum lateral distance used to classify a short obstacle (m).
        self.max_front_length = float(
            self.declare_parameter('max_front_length', 0.50).value
        )
        # Forward velocity (m/s).
        self.forward_speed = float(
            self.declare_parameter('forward_speed', 0.20).value
        )
        # Maximum lateral velocity (m/s).
        self.lateral_speed = float(
            self.declare_parameter('lateral_speed', 0.20).value
        )
        # Turn speed and maximum angular correction (rad/s).
        self.turn_speed = float(
            self.declare_parameter('turn_speed', 1.00).value
        )
        # Maximum LiDAR distance considered (m).
        self.detection_limit = float(
            self.declare_parameter('detection_limit', 1.20).value
        )
        # Proportional gain for lateral-distance correction.
        self.lateral_gain = float(
            self.declare_parameter('lateral_gain', 2.0).value
        )
        # Proportional gain for angular alignment.
        self.alignment_gain = float(
            self.declare_parameter('alignment_gain', 2.0).value
        )

        self.RIGHT = 'RIGHT'
        self.FRONT = 'FRONT'
        self.FRONT_RIGHT = 'FRONT_RIGHT'
        self.BACK_RIGHT = 'BACK_RIGHT'

        self.diagonal_factor = 0.5
        self.region_confirmations = 3
        self.robust_rays = 3

        self.state = self.RIGHT
        self.obstacle_is_long = False
        self.passing_short_obstacle = False
        self.command = Twist()
        self.odom_pose = None
        self.front_start_pose = None
        self.right_angle = None

        self.region_candidate = None
        self.region_candidate_count = 0
        self.stable_region = None

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info(
            'Student template started. Complete the TODO sections.'
        )

    def publish_command(self):
        self.publisher.publish(self.command)

    def odom_callback(self, msg):
        """Store planar pose (x, y, yaw) from odometry."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.odom_pose = (position.x, position.y, yaw)

    def region_measurements(self, scan):
        """Return robust distances and angles for the four LiDAR regions."""

        # TODO 1: Create the four LiDAR regions.
        regions = {
            self.FRONT: [],
            self.FRONT_RIGHT: [],
            self.RIGHT: [],
            self.BACK_RIGHT: [],
        }

        # Process every LiDAR ray.
        for i, distance in enumerate(scan.ranges):

            # Ignore invalid or out-of-range measurements.
            if not math.isfinite(distance):
                continue

            if distance < scan.range_min or distance > scan.range_max:
                continue

            # Calculate the angle of this ray in degrees.
            angle_rad = scan.angle_min + i * scan.angle_increment
            angle_deg = math.degrees(angle_rad)

            # Classify the ray into its corresponding region.
            if -20 <= angle_deg <= 20:
                regions[self.FRONT].append((distance, angle_deg))

            elif -70 <= angle_deg < -20:
                regions[self.FRONT_RIGHT].append((distance, angle_deg))

            elif -110 <= angle_deg < -70:
                regions[self.RIGHT].append((distance, angle_deg))

            elif -160 <= angle_deg < -110:
                regions[self.BACK_RIGHT].append((distance, angle_deg))

        # TODO 2: Calculate robust distance and angle measurements.
        measurements = {}

        for region, rays in regions.items():

            # Select the three nearest valid rays.
            nearest = sorted(
                rays,
                key=lambda ray: ray[0]
            )[:self.robust_rays]

            if nearest:
                mean_distance = sum(ray[0] for ray in nearest) / len(nearest)
                mean_angle = sum(ray[1] for ray in nearest) / len(nearest)

                measurements[region] = (mean_distance, mean_angle)

            else:
                measurements[region] = (math.inf, None)

        return measurements


    def closest_region(self, measurements):
        """Return the closest region inside detection_limit."""

        closest = None
        closest_distance = self.detection_limit

        for region, (distance, angle) in measurements.items():

            if distance < closest_distance:
                closest_distance = distance
                closest = region

        return closest


    def confirmed_region(self, measurements):
        """Confirm a region change over several consecutive scans."""

        current_region = self.closest_region(measurements)

        # If the detected region matches the stable one,
        # cancel any pending region change.
        if current_region == self.stable_region:
            self.region_candidate = None
            self.region_candidate_count = 0
            return self.stable_region

        # A different candidate has appeared.
        if current_region != self.region_candidate:
            self.region_candidate = current_region
            self.region_candidate_count = 1

        # The same candidate appeared in another scan.
        else:
            self.region_candidate_count += 1

        # Accept the new region after three consecutive scans.
        if self.region_candidate_count >= self.region_confirmations:
            self.stable_region = self.region_candidate
            self.region_candidate = None
            self.region_candidate_count = 0

        return self.stable_region

    def lateral_progress(self):
        """Return travel along the robot's initial lateral axis."""

        # We need both the initial and current odometry poses.
        if self.front_start_pose is None or self.odom_pose is None:
            return 0.0

        # Pose when the robot entered the FRONT state.
        x0, y0, yaw0 = self.front_start_pose

        # Current robot pose.
        x, y, _ = self.odom_pose

        # Displacement in the global odometry frame.
        dx = x - x0
        dy = y - y0

        # Project displacement onto the initial lateral axis.
        progress = abs(
            -math.sin(yaw0) * dx
            + math.cos(yaw0) * dy
        )

        return progress


    def follow_right(self, measurements, closest, cmd):
        """Follow the wall using lateral and angular correction."""

        # TODO 6A: A frontal obstacle has been detected.
        if closest == self.FRONT:

            # Stop moving forward.
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

            # Odometry is required before starting obstacle avoidance.
            if self.odom_pose is not None:
                self.state = self.FRONT

                # Save the pose at the beginning of the manoeuvre.
                self.front_start_pose = self.odom_pose

                # Reset the obstacle classification.
                self.obstacle_is_long = False
                self.passing_short_obstacle = False

            return

        # TODO 6B: During a short-obstacle bypass,
        # transition to BACK_RIGHT when the corner moves behind.
        if self.passing_short_obstacle and closest == self.BACK_RIGHT:
            self.state = self.BACK_RIGHT
            return

        # Read the right-wall distance and angle.
        right_distance, right_angle = measurements[self.RIGHT]

        # Store the current angular measurement.
        self.right_angle = right_angle

        # Do not move forward if the right wall is not detected.
        if (
            not math.isfinite(right_distance)
            or right_distance >= self.detection_limit
            or right_angle is None
        ):
            return

        # TODO 6C: Move forward.
        cmd.linear.x = self.forward_speed

        # TODO 6D: Correct the distance to the right wall.
        distance_error = right_distance - self.follow_distance

        cmd.linear.y = clamp(
            -self.lateral_gain * distance_error,
            -self.lateral_speed,
            self.lateral_speed
        )

        # TODO 6E: Correct the angular alignment.
        if self.passing_short_obstacle:

            # The closest ray may belong to an obstacle corner.
            # Do not rotate while passing it.
            cmd.angular.z = 0.0

        else:

            # Desired RIGHT angle is -90 degrees.
            angle_error_deg = right_angle + 90.0
            angle_error_rad = math.radians(angle_error_deg)

            cmd.angular.z = clamp(
                self.alignment_gain * angle_error_rad,
                -self.turn_speed,
                self.turn_speed
            )



    def follow_front(self, measurements, closest, cmd):
        """Move laterally and classify the obstacle as short or long."""

        # Do not begin the manoeuvre without odometry.
        if self.front_start_pose is None or self.odom_pose is None:
            return

        # Measure lateral displacement since entering FRONT.
        progress = self.lateral_progress()

        # Obtain the current frontal distance.
        front_distance, _ = measurements[self.FRONT]

        # FRONT is clear when no obstacle is detected within
        # the relevant detection distance.
        front_is_clear = front_distance >= self.detection_limit

        # CASE 1: Short obstacle.
        if (
            front_is_clear
            and closest != self.FRONT
            and progress < self.max_front_length
        ):
            self.obstacle_is_long = False
            self.passing_short_obstacle = True
            self.state = self.FRONT_RIGHT
            return

        # CASE 2: Long obstacle.
        if progress >= self.max_front_length:
            self.obstacle_is_long = True
            self.passing_short_obstacle = False
            self.state = self.FRONT_RIGHT
            return

        # CASE 3: Continue moving laterally to the left.
        cmd.linear.x = 0.0
        cmd.linear.y = self.lateral_speed
        cmd.angular.z = 0.0



    def follow_front_right(self, measurements, closest, cmd):
        """Move diagonally around a short obstacle or turn along a long wall."""

        # CASE 1: Long wall.
        if self.obstacle_is_long:

            # Rotate left in place.
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = self.turn_speed

            if self.odom_pose is None or self.front_start_pose is None:
                cmd.angular.z = 0.0
                return

            # Measure how much the robot has turned.
            yaw0 = self.front_start_pose[2]
            yaw = self.odom_pose[2]

            angle_turned = math.atan2(
                math.sin(yaw - yaw0),
                math.cos(yaw - yaw0)
            )

            front_distance, _ = measurements[self.FRONT]

            # A confirmed RIGHT detection after sufficient rotation
            # indicates the frontal wall is now on the robot's right.
            if (
                angle_turned >= math.pi / 4
                and closest == self.RIGHT
                and front_distance >= self.follow_distance
            ):
                self.state = self.RIGHT
                self.obstacle_is_long = False
                self.front_start_pose = None
                cmd.angular.z = 0.0

            return

        # CASE 2: Short obstacle.

        # If the frontal obstacle reappears, return to lateral avoidance.
        if closest == self.FRONT:
            self.state = self.FRONT
            self.passing_short_obstacle = False
            cmd.linear.y = self.lateral_speed
            return

        # Stop if no relevant obstacle or wall is detected.
        if closest is None:
            return

        # Move diagonally forward-left.
        cmd.linear.x = self.forward_speed * self.diagonal_factor
        cmd.linear.y = self.lateral_speed * self.diagonal_factor
        cmd.angular.z = 0.0

        # When the obstacle reaches RIGHT, enter the temporary
        # wall-following phase without angular alignment.
        if closest == self.RIGHT:
            self.state = self.RIGHT


    def follow_back_right(self, measurements, closest, cmd):
        """Complete the bypass and recover the right wall."""

        # CASE 1: A frontal obstacle blocks the path again.
        if closest == self.FRONT:
            self.state = self.FRONT
            self.front_start_pose = self.odom_pose
            self.obstacle_is_long = False
            self.passing_short_obstacle = False
            return

        # CASE 2: The right wall has been recovered.
        if closest == self.RIGHT:

            # Return to normal wall following.
            self.state = self.RIGHT

            # Enable angular alignment again.
            self.passing_short_obstacle = False
            self.obstacle_is_long = False
            self.front_start_pose = None

            return

        # CASE 3: Continue moving around the obstacle.
        back_right_distance, _ = measurements[self.BACK_RIGHT]
        right_distance, _ = measurements[self.RIGHT]

        # Move forward slowly.
        cmd.linear.x = self.forward_speed * self.diagonal_factor

        # Only approach the right side if there is enough space.
        if (
            back_right_distance < self.follow_distance
            or right_distance < self.follow_distance
        ):
            cmd.linear.y = 0.0

        else:
            # Move diagonally forward-right.
            cmd.linear.y = (
                -self.lateral_speed * self.diagonal_factor
            )

        # Keep the robot's orientation unchanged.
        cmd.angular.z = 0.0



    def scan_callback(self, scan):
        """Read the LiDAR and calculate the next velocity command."""

        # TODO 10A: Process the LiDAR measurements.
        measurements = self.region_measurements(scan)

        # TODO 10B: Confirm the closest region.
        closest = self.confirmed_region(measurements)

        # TODO 10C: Create a new velocity command.
        cmd = Twist()

        # Do not move until odometry is available.
        if self.odom_pose is None:
            self.command = cmd
            return

        # Remember the previous state for debugging.
        previous_state = self.state

        # TODO 10D: Execute the current state.
        if self.state == self.RIGHT:
            self.follow_right(measurements, closest, cmd)

        elif self.state == self.FRONT:
            self.follow_front(measurements, closest, cmd)

        elif self.state == self.FRONT_RIGHT:
            self.follow_front_right(measurements, closest, cmd)

        elif self.state == self.BACK_RIGHT:
            self.follow_back_right(measurements, closest, cmd)

        else:
            self.get_logger().warn(
                f'Unknown state: {self.state}'
            )

        # TODO 10E: Save the velocity command.
        self.command = cmd

        # Log state transitions for debugging.
        if self.state != previous_state:
            self.get_logger().info(
                f'State transition: '
                f'{previous_state} -> {self.state} '
                f'(closest: {closest})'
            )

    def stop(self):
        """Publish a final zero command."""
        self.command = Twist()
        self.timer.cancel()
        try:
            self.publisher.publish(self.command)
        except Exception:
            pass


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = HolonomicObstacleWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

