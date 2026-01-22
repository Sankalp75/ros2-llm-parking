#!/usr/bin/env python3
"""
Parking Controller Node for Ackermann Vehicle
Implements parking maneuvers: parallel, perpendicular, simple, and reverse parking.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from vehicle_interfaces.action import ParkingCommand
from vehicle_interfaces.msg import ParkingStatus

import math
import time
from typing import Optional, Tuple
from enum import Enum


class ParkingPhase(Enum):
    """Phases of parking maneuver."""
    APPROACHING = "approaching"
    POSITIONING = "positioning"
    TURNING = "turning"
    REVERSING = "reversing"
    ALIGNING = "aligning"
    FINAL_ADJUSTMENT = "final_adjustment"
    COMPLETE = "complete"


class ParkingControllerNode(Node):
    """ROS 2 node for executing parking maneuvers on Ackermann vehicle."""
    
    # Vehicle parameters (should match URDF)
    WHEELBASE = 1.8  # meters
    TRACK_WIDTH = 1.4  # meters
    MAX_STEERING_ANGLE = 0.6  # radians
    
    # Control parameters
    LINEAR_SPEED = 2.0  # m/s (Realistic driving speed)
    ANGULAR_SPEED = 0.8  # rad/s
    POSITION_TOLERANCE = 1.0  # meters (more forgiving)
    ANGLE_TOLERANCE = 0.2  # radians
    
    def __init__(self):
        super().__init__('parking_controller_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('position_tolerance', 0.5)
        self.declare_parameter('angle_tolerance', 0.1)
        
        self.LINEAR_SPEED = self.get_parameter('linear_speed').value
        self.POSITION_TOLERANCE = self.get_parameter('position_tolerance').value
        self.ANGLE_TOLERANCE = self.get_parameter('angle_tolerance').value
        
        # Callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # State
        self.current_pose: Optional[Pose] = None
        self.current_yaw: float = 0.0
        self.min_obstacle_distance: float = float('inf')
        self.is_executing = False
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(ParkingStatus, '/parking_status', 10)
        
        # Action server
        self.parking_action_server = ActionServer(
            self,
            ParkingCommand,
            '/execute_parking',
            execute_callback=self.execute_parking_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Parking Controller initialized')
        self.publish_status('idle', '', 0.0, '')
    
    def goal_callback(self, goal_request):
        """Handle incoming goal requests."""
        self.get_logger().info(f'Received parking goal: {goal_request.parking_type}')
        
        if self.is_executing:
            self.get_logger().warn('Already executing a parking maneuver')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle cancel requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def odom_callback(self, msg: Odometry):
        """Update current pose from odometry."""
        self.current_pose = msg.pose.pose
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_callback(self, msg: LaserScan):
        """Update minimum obstacle distance from lidar."""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.min_obstacle_distance = min(valid_ranges)
        else:
            self.min_obstacle_distance = float('inf')
    
    def publish_status(self, status: str, parking_type: str, progress: float, phase: str):
        """Publish parking status."""
        msg = ParkingStatus()
        msg.status = status
        msg.parking_type = parking_type
        msg.progress_percent = float(progress)
        msg.current_phase = phase
        self.status_pub.publish(msg)
    
    def stop_vehicle(self):
        """Send zero velocity command."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def send_velocity(self, linear: float, angular: float):
        """Send velocity command to vehicle."""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)
    
    def get_distance_to_target(self, target: Pose) -> float:
        """Calculate distance to target position."""
        if self.current_pose is None:
            return float('inf')
        
        dx = target.position.x - self.current_pose.position.x
        dy = target.position.y - self.current_pose.position.y
        return math.sqrt(dx * dx + dy * dy)
    
    def get_angle_to_target(self, target: Pose) -> float:
        """Calculate angle to target position."""
        if self.current_pose is None:
            return 0.0
        
        dx = target.position.x - self.current_pose.position.x
        dy = target.position.y - self.current_pose.position.y
        target_angle = math.atan2(dy, dx)
        
        angle_diff = target_angle - self.current_yaw
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff
    
    def get_target_yaw(self, target: Pose) -> float:
        """Extract yaw from target pose quaternion."""
        q = target.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def get_yaw_error(self, target: Pose) -> float:
        """Calculate yaw error to target orientation."""
        target_yaw = self.get_target_yaw(target)
        error = target_yaw - self.current_yaw
        
        # Normalize to [-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        return error
    
    async def execute_parking_callback(self, goal_handle):
        """Execute the parking maneuver."""
        self.is_executing = True
        
        parking_type = goal_handle.request.parking_type
        target_pose = goal_handle.request.target_pose
        speed = goal_handle.request.speed if goal_handle.request.speed > 0 else self.LINEAR_SPEED
        
        self.get_logger().info(f'Executing {parking_type} parking at ({target_pose.position.x}, {target_pose.position.y})')
        
        result = ParkingCommand.Result()
        feedback = ParkingCommand.Feedback()
        
        try:
            # Wait for odometry
            timeout = 5.0
            start_time = time.time()
            while self.current_pose is None and time.time() - start_time < timeout:
                self.get_logger().info('Waiting for odometry...')
                time.sleep(0.1)
            
            if self.current_pose is None:
                result.success = False
                result.message = 'No odometry received'
                self.stop_vehicle()
                time.sleep(1.0)

                self.is_executing = False
                goal_handle.abort()
                return result
            
            # Execute based on parking type
            if parking_type == 'parallel':
                success = await self.execute_parallel_parking(goal_handle, target_pose, speed, feedback)
            elif parking_type == 'perpendicular':
                success = await self.execute_perpendicular_parking(goal_handle, target_pose, speed, feedback)
            elif parking_type == 'reverse':
                success = await self.execute_reverse_parking(goal_handle, target_pose, speed, feedback)
            else:  # simple
                success = await self.execute_simple_parking(goal_handle, target_pose, speed, feedback)
            
            # Final result
            self.stop_vehicle()

            
            result.success = success
            result.message = 'Parking complete' if success else 'Parking failed'
            result.final_distance_error = self.get_distance_to_target(target_pose)
            result.final_angle_error = abs(self.get_yaw_error(target_pose))
            
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            
            self.publish_status('parked' if success else 'failed', parking_type, 100.0, 'complete')
            
        except Exception as e:
            self.get_logger().error(f'Parking error: {e}')
            result.success = False
            result.message = str(e)
            self.stop_vehicle()

            goal_handle.abort()
        
        finally:
            self.is_executing = False
        
        return result
    
    async def execute_simple_parking(self, goal_handle, target: Pose, speed: float, feedback):
        """Execute simple forward parking to target."""
        self.get_logger().info('Executing simple parking')
        
        rate = self.create_rate(10)
        
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            
            distance = self.get_distance_to_target(target)
            angle_to_target = self.get_angle_to_target(target)
            
            # Update feedback
            progress = max(0, min(100, (1 - distance / 10.0) * 100))
            feedback.progress_percent = float(progress)
            feedback.current_phase = ParkingPhase.APPROACHING.value
            feedback.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback)
            
            self.publish_status('parking', 'simple', progress, 'approaching')
            
            # Check if reached target
            if distance < self.POSITION_TOLERANCE:
                self.get_logger().info('Reached target position')
                break
            
            # Calculate control
            linear_vel = min(speed, distance * 0.5)
            angular_vel = angle_to_target * 2.0
            angular_vel = max(-self.ANGULAR_SPEED, min(self.ANGULAR_SPEED, angular_vel))
            
            # (Obstacle slow-down removed for Ludicrous speed demo)
            
            self.send_velocity(linear_vel, angular_vel)
            
            try:
                rate.sleep()
            except:
                time.sleep(0.1)
        
        self.stop_vehicle()

        return True
    
    async def execute_perpendicular_parking(self, goal_handle, target: Pose, speed: float, feedback):
        """Execute perpendicular parking maneuver."""
        self.get_logger().info('Executing perpendicular parking')
        
        rate = self.create_rate(10)
        
        # Phase 1: Drive parallel to parking spot
        approach_point = Pose()
        approach_point.position = Point(
            x=target.position.x - 4.0,  # 4m before the spot
            y=target.position.y,
            z=0.0
        )
        approach_point.orientation = target.orientation
        
        feedback.current_phase = ParkingPhase.APPROACHING.value
        
        while self.get_distance_to_target(approach_point) > self.POSITION_TOLERANCE:
            if goal_handle.is_cancel_requested:
                return False
            
            distance = self.get_distance_to_target(approach_point)
            angle = self.get_angle_to_target(approach_point)
            
            progress = max(0, min(30, (1 - distance / 10.0) * 30))
            feedback.progress_percent = float(progress)
            feedback.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback)
            
            linear_vel = min(speed, distance * 1.0)
            angular_vel = max(-self.ANGULAR_SPEED, min(self.ANGULAR_SPEED, angle * 2.0))
            
            self.send_velocity(linear_vel, angular_vel)
            
            try:
                rate.sleep()
            except:
                time.sleep(0.1)
        
        self.stop_vehicle()

        
        # Phase 2: Turn toward the parking spot
        target_yaw = self.get_target_yaw(target)
        feedback.current_phase = ParkingPhase.TURNING.value
        
        while abs(self.get_yaw_error(target)) > self.ANGLE_TOLERANCE:
            if goal_handle.is_cancel_requested:
                return False
            
            yaw_error = self.get_yaw_error(target)
            
            progress = 30 + max(0, min(30, (1 - abs(yaw_error) / math.pi) * 30))
            feedback.progress_percent = float(progress)
            feedback.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback)
            
            angular_vel = max(-self.ANGULAR_SPEED, min(self.ANGULAR_SPEED, yaw_error * 0.8))
            self.send_velocity(0.1, angular_vel)  # Slow forward while turning
            
            try:
                rate.sleep()
            except:
                time.sleep(0.1)
        
        self.stop_vehicle()

        
        
        # Phase 3: Drive into the spot
        feedback.current_phase = ParkingPhase.FINAL_ADJUSTMENT.value
        
        while self.get_distance_to_target(target) > self.POSITION_TOLERANCE:
            if goal_handle.is_cancel_requested:
                return False
            
            distance = self.get_distance_to_target(target)
            angle = self.get_angle_to_target(target)
            yaw_error = self.get_yaw_error(target)
            
            progress = 60 + max(0, min(40, (1 - distance / 5.0) * 40))
            feedback.progress_percent = float(progress)
            feedback.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback)
            
            linear_vel = min(speed * 0.5, distance * 0.5)
            angular_vel = max(-self.ANGULAR_SPEED * 0.5, min(self.ANGULAR_SPEED * 0.5, yaw_error * 1.5))
            
            # Check for obstacles
            if self.min_obstacle_distance < 0.5:
                self.get_logger().warn('Too close to obstacle, stopping')
                break
            
            self.send_velocity(linear_vel, angular_vel)
            
            try:
                rate.sleep()
            except:
                time.sleep(0.1)
        
        self.stop_vehicle()

        return True
    
    async def execute_parallel_parking(self, goal_handle, target: Pose, speed: float, feedback):
        """Execute parallel parking maneuver."""
        self.get_logger().info('Executing parallel parking')
        
        rate = self.create_rate(10)
        
        # Phase 1: Drive past the parking spot
        pass_point = Pose()
        pass_point.position = Point(
            x=target.position.x + 3.0,  # 3m past the spot
            y=target.position.y - 1.5,  # Offset to the side
            z=0.0
        )
        pass_point.orientation = target.orientation
        
        feedback.current_phase = ParkingPhase.POSITIONING.value
        
        while self.get_distance_to_target(pass_point) > self.POSITION_TOLERANCE:
            if goal_handle.is_cancel_requested:
                return False
            
            distance = self.get_distance_to_target(pass_point)
            angle = self.get_angle_to_target(pass_point)
            
            progress = max(0, min(25, (1 - distance / 10.0) * 25))
            feedback.progress_percent = float(progress)
            feedback.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback)
            
            linear_vel = min(speed, distance * 1.0)
            angular_vel = max(-self.ANGULAR_SPEED, min(self.ANGULAR_SPEED, angle * 2.0))
            
            self.send_velocity(linear_vel, angular_vel)
            
            try:
                rate.sleep()
            except:
                time.sleep(0.1)
        
        self.stop_vehicle()

        
        
        # Phase 2: Turn wheels and reverse into spot (first arc)
        feedback.current_phase = ParkingPhase.REVERSING.value
        
        # Reverse with right turn
        turn_duration = 3.0
        start_time = time.time()
        
        while time.time() - start_time < turn_duration:
            if goal_handle.is_cancel_requested:
                return False
            
            elapsed = time.time() - start_time
            progress = 25 + (elapsed / turn_duration) * 25
            feedback.progress_percent = float(progress)
            feedback.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback)
            
            # High speed reverse
            self.send_velocity(-speed, -self.ANGULAR_SPEED)
            
            if self.min_obstacle_distance < 0.3:
                self.get_logger().warn('Obstacle detected during reverse')
                break
            
            try:
                rate.sleep()
            except:
                time.sleep(0.1)
        
        self.stop_vehicle()

        
        
        # Phase 3: Straighten and continue reverse (second arc)
        feedback.current_phase = ParkingPhase.ALIGNING.value
        
        start_time = time.time()
        while time.time() - start_time < turn_duration:
            if goal_handle.is_cancel_requested:
                return False
            
            elapsed = time.time() - start_time
            progress = 50 + (elapsed / turn_duration) * 25
            feedback.progress_percent = float(progress)
            feedback.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback)
            
            # Reverse with opposite steering to straighten
            self.send_velocity(-speed * 0.3, self.ANGULAR_SPEED * 0.6)
            
            if self.min_obstacle_distance < 0.3:
                break
            
            try:
                rate.sleep()
            except:
                time.sleep(0.1)
        
        self.stop_vehicle()

        
        
        # Phase 4: Final adjustments
        feedback.current_phase = ParkingPhase.FINAL_ADJUSTMENT.value
        
        # Center in the spot
        iterations = 0
        max_iterations = 50
        
        while self.get_distance_to_target(target) > self.POSITION_TOLERANCE * 2 and iterations < max_iterations:
            if goal_handle.is_cancel_requested:
                return False
            
            iterations += 1
            distance = self.get_distance_to_target(target)
            yaw_error = self.get_yaw_error(target)
            
            progress = 75 + min(25, iterations / max_iterations * 25)
            feedback.progress_percent = float(progress)
            feedback.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback)
            
            # Small adjustments
            if distance > self.POSITION_TOLERANCE:
                direction = 1 if self.current_pose.position.x < target.position.x else -1
                self.send_velocity(direction * speed * 0.5, yaw_error * 0.3)
            
            try:
                rate.sleep()
            except:
                time.sleep(0.1)
        
        self.stop_vehicle()

        return True
    
    async def execute_reverse_parking(self, goal_handle, target: Pose, speed: float, feedback):
        """Execute reverse parking (back into spot)."""
        self.get_logger().info('Executing reverse parking')
        
        rate = self.create_rate(10)
        
        # Phase 1: Position for reverse
        prep_point = Pose()
        prep_point.position = Point(
            x=target.position.x - 5.0,
            y=target.position.y,
            z=0.0
        )
        target_yaw = self.get_target_yaw(target)
        prep_point.orientation = target.orientation
        
        feedback.current_phase = ParkingPhase.POSITIONING.value
        
        while self.get_distance_to_target(prep_point) > self.POSITION_TOLERANCE:
            if goal_handle.is_cancel_requested:
                return False
            
            distance = self.get_distance_to_target(prep_point)
            angle = self.get_angle_to_target(prep_point)
            
            progress = max(0, min(30, (1 - distance / 10.0) * 30))
            feedback.progress_percent = float(progress)
            feedback.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback)
            
            linear_vel = min(speed, distance * 1.0)
            angular_vel = max(-self.ANGULAR_SPEED, min(self.ANGULAR_SPEED, angle * 2.0))
            
            self.send_velocity(linear_vel, angular_vel)
            
            try:
                rate.sleep()
            except:
                time.sleep(0.1)
        
        self.stop_vehicle()

        
        
        # Phase 2: Reverse into spot
        feedback.current_phase = ParkingPhase.REVERSING.value
        
        while self.get_distance_to_target(target) > self.POSITION_TOLERANCE:
            if goal_handle.is_cancel_requested:
                return False
            
            distance = self.get_distance_to_target(target)
            yaw_error = self.get_yaw_error(target)
            
            # Calculate reverse angle (opposite direction)
            dx = target.position.x - self.current_pose.position.x
            dy = target.position.y - self.current_pose.position.y
            reverse_angle = math.atan2(dy, dx) - math.pi
            angle_diff = reverse_angle - self.current_yaw
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            progress = 30 + max(0, min(70, (1 - distance / 6.0) * 70))
            feedback.progress_percent = float(progress)
            feedback.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback)
            
            linear_vel = -min(speed * 0.5, distance * 0.3)
            angular_vel = max(-self.ANGULAR_SPEED, min(self.ANGULAR_SPEED, -angle_diff * 0.5))
            
            if self.min_obstacle_distance < 0.4:
                self.get_logger().warn('Obstacle behind, stopping')
                break
            
            self.send_velocity(linear_vel, angular_vel)
            
            try:
                rate.sleep()
            except:
                time.sleep(0.1)
        
        self.stop_vehicle()

        return True


def main(args=None):
    rclpy.init(args=args)
    
    node = ParkingControllerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
