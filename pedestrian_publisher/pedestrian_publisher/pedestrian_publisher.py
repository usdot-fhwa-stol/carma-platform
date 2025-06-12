#!/usr/bin/env python3
# pedestrian_publisher.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion, Twist, Vector3, Vector3 as TwistVector3
import math
import time
import threading

# Import the ExternalObject message
from carma_perception_msgs.msg import ExternalObject, ExternalObjectList
from carma_planning_msgs.msg import GuidanceState

class PedestrianPublisher(Node):
    def __init__(self):
        super().__init__('pedestrian_publisher')

        # Create a QoS profile for reliable publishing and subscribing
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publisher for the external object list
        self.publisher = self.create_publisher(
            ExternalObjectList,
            '/environment/external_objects',
            qos
        )

        # Temporary disable the initial pose subscriber for 4/24 demo
        # Subscriber to get the starting pose and direction
        # self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     '/localization/initialpose',
        #     self.initialpose_callback,
        #     qos
        # )

        # Subscriber to monitor current pose for reset condition
        self.create_subscription(
            PoseStamped,
            '/localization/current_pose',
            self.current_pose_callback,
            qos
        )

        self.create_subscription(
            GuidanceState,
            '/guidance/state',
            self.guidance_state_callback,
            qos
        )

        # Parameters for pedestrian configuration
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('start_x', -21.112)
        self.declare_parameter('start_y', 309.947)
        self.declare_parameter('end_x', 45.6094627)
        self.declare_parameter('end_y', 413.60025)

        # Get configured values
        self.start_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.start_y = self.get_parameter('start_y').get_parameter_value().double_value
        self.end_x = self.get_parameter('end_x').get_parameter_value().double_value
        self.end_y = self.get_parameter('end_y').get_parameter_value().double_value

        # X, Y position threshold for reset
        self.reset_threshold_x = -42.307
        self.reset_threshold_y = 331.909 # helps avoid triggering when 0 initialization
        #Guidance state
        self.is_engaged = False

        # Initialize pedestrian at configured starting position
        self.initialize_pedestrian()

        # Use time-based update for smooth movement
        self.last_update_time = time.time()
        self.initialized_based_on_vehicle = False
        # Create a timer that always updates the pedestrian's position
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / update_rate, self.timer_callback)

        # Thread lock for thread safety
        self.lock = threading.Lock()

        self.get_logger().info('Pedestrian publisher node initialized with configured start position.')
        self.get_logger().info(f'Start position: ({self.start_x}, {self.start_y}), End position: ({self.end_x}, {self.end_y})')
        self.get_logger().info(f'Reset threshold x: {self.reset_threshold_x}')

    def initialize_pedestrian(self):
        """Initialize the pedestrian at the configured starting position"""
        self.current_x = self.start_x
        self.current_y = self.start_y

        # Calculate direction vector from start to end
        dx = self.end_x - self.start_x
        dy = self.end_y - self.start_y
        distance = math.sqrt(dx**2 + dy**2)

        # Normalize direction vector
        if distance > 0:
            self.direction_x = dx / distance
            self.direction_y = dy / distance
        else:
            self.direction_x = 0.0
            self.direction_y = 1.0  # Default direction if start and end are the same

        # Calculate yaw from direction vector
        self.yaw = math.atan2(self.direction_y, self.direction_x)

        self.get_logger().info(f"Pedestrian initialized at ({self.current_x:.2f}, {self.current_y:.2f}) with yaw: {self.yaw:.2f} radians.")

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback that gets triggered whenever a new initial pose is published."""
        with self.lock:
            # Update current pose from the message
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y

            # Convert quaternion to yaw
            self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

            # Compute direction vector from yaw
            self.direction_x = math.cos(self.yaw)
            self.direction_y = math.sin(self.yaw)

            # Reset the last update time so dt is small on the next update
            self.last_update_time = time.time()

            self.get_logger().info(
                f"Received new initial pose: ({self.current_x:.2f}, {self.current_y:.2f}) with yaw: {self.yaw:.2f} radians."
            )

    def current_pose_callback(self, msg: PoseStamped):
        """Callback to monitor current vehicle pose for reset condition"""
        vehicle_x = msg.pose.position.x
        vehicle_y = msg.pose.position.y

        if self.initialized_based_on_vehicle:
            return
        # If vehicle x position is above the threshold, reset pedestrian
        if abs(vehicle_x) > abs(self.reset_threshold_x) and vehicle_y < self.reset_threshold_y and self.is_engaged:
            with self.lock:
                self.get_logger().info(f"Vehicle x position {vehicle_x:.2f} exceeded threshold {self.reset_threshold_x:.2f}, resetting pedestrian")
                self.initialize_pedestrian()
                self.last_update_time = time.time()
                self.initialized_based_on_vehicle = True

    def guidance_state_callback(self, msg: GuidanceState):
        """Callback to monitor guidance state for reset condition"""
        state = msg.state
        if state == 4:
            self.is_engaged = True

    def timer_callback(self):
        """Timer callback to update and publish the pedestrian position infinitely."""
        with self.lock:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time

            # Get the speed from parameter
            speed = self.get_parameter('speed').get_parameter_value().double_value
            # Update position based on the current direction and speed
            self.current_x += self.direction_x * speed * dt
            self.current_y += self.direction_y * speed * dt

            # Publish the pedestrian position as an ExternalObject
            self.publish_pedestrian()

    def publish_pedestrian(self):
        """Create and publish the ExternalObject message for the pedestrian."""
        list_msg = ExternalObjectList()
        # Set the header
        list_msg.header = Header()
        list_msg.header.stamp = self.get_clock().now().to_msg()
        list_msg.header.frame_id = "map"  # Using map frame as specified

        if self.is_engaged:
            msg = ExternalObject()
            # Set the header
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            # Set presence vector flags (using fixed flags for simulation)
            msg.presence_vector = (
                ExternalObject.ID_PRESENCE_VECTOR |
                ExternalObject.POSE_PRESENCE_VECTOR |
                ExternalObject.VELOCITY_PRESENCE_VECTOR |
                ExternalObject.SIZE_PRESENCE_VECTOR |
                ExternalObject.OBJECT_TYPE_PRESENCE_VECTOR |
                ExternalObject.DYNAMIC_OBJ_PRESENCE
            )

            # Fixed ID for the pedestrian
            msg.id = 1001

            # Set pose information
            msg.pose = PoseWithCovariance()
            msg.pose.pose = Pose()
            msg.pose.pose.position = Point(x=self.current_x, y=self.current_y, z=0.0)

            # Convert yaw to quaternion for orientation
            q = self.euler_to_quaternion(0.0, 0.0, self.yaw)
            msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            # Hardcode covariance for the CP to work
            # We need pose_covariance_x, pose_covariance_y, pose_covariance_z
            # and pose_covariance_yaw, twist_covariance_x, twist_covariance_z, twist_covariance_yaw
            msg.pose.covariance = [0.0 for i in range(36)]

            msg.pose.covariance[0] = 0.02 # X-axis position uncertainty (lateral)
            msg.pose.covariance[7] = 0.02 # Y-axis position uncertainty (longitudinal)
            msg.pose.covariance[14] = 0.02 # Z-axis position uncertainty (vertical)
            msg.pose.covariance[35] = 0.394384 # 10% error aw, which is (0.1 * 2 pi) ^ 2 = 0.394384

            # Set velocity information
            msg.velocity = TwistWithCovariance()
            msg.velocity.twist = Twist()
            msg.velocity.twist.linear = TwistVector3(
                x=self.direction_x * self.get_parameter('speed').get_parameter_value().double_value,
                y=self.direction_y * self.get_parameter('speed').get_parameter_value().double_value,
                z=0.0
            )
            msg.velocity.covariance = [0.0 for i in range(36)]

            msg.velocity.covariance[0] = 0.005 # X-axis velocity uncertainty
            msg.velocity.covariance[14] = 0.005 # Z-axis velocity uncertainty
            msg.velocity.covariance[35] = 0.005 # Yaw angular velocity uncertainty

            # For this simulation, instantaneous velocity is the same as average velocity
            msg.velocity_inst = msg.velocity

            # Set pedestrian dimensions (typical dimensions)
            msg.size = Vector3(x=0.5, y=0.5, z=1.8)

            # Confidence and object type
            msg.confidence = 1.0
            msg.object_type = ExternalObject.PEDESTRIAN

            # Mark the object as dynamic
            msg.dynamic_obj = True

            list_msg.objects = [msg]
        else:
            # If not engaged, publish an empty list
            list_msg.objects = []
        self.publisher.publish(list_msg)

    def quaternion_to_yaw(self, q: Quaternion) -> float:
        """Convert a quaternion into yaw (rotation about the z-axis)."""
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """Convert Euler angles (roll, pitch, yaw) to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    pedestrian_publisher = PedestrianPublisher()
    try:
        rclpy.spin(pedestrian_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        pedestrian_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
