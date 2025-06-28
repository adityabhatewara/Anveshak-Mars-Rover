import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from nav_msgs.msg import Odometry


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__("emergency_stop_node")
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.subscription = self.create_subscription(
            LaserScan, "/bcr_bot/scan", self.vel_callback, qos_profile)
        
        self.publisher = self.create_publisher(
            Twist, "/bcr_bot/cmd_vel", 10)
        self.Odometry = self.create_subscription(
            Odometry, "/bcr_bot/odom", self.Odom_callback, qos_profile)
        
        # inside __init__()
        self.angle_start = -np.pi / 6 # -30 degrees
        self.angle_end = np.pi / 6     # +30 degrees


        self.prev_dist = None
        self.last_time = 0.0
        self.ittc_threshold = 1.6
        self.current_vel = 0.0
        

    def Odom_callback(self,msg):
        self.current_vel = msg.twist.twist.linear.x
        # self.get_logger().info(f"Received velocity: {self.current_vel:.3f}")


    def vel_callback(self,msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        # Full preprocessed LIDAR array
        ranges_full = np.array(msg.ranges)
        ranges_full = np.where(np.isfinite(ranges_full), ranges_full, 10.0)


        # # Compute angles per index
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angles = angle_min + np.arange(len(ranges_full)) * angle_increment

        # # Normalize angles to [-π, π]
        angles = (angles + np.pi) % (2 * np.pi) - np.pi

        # # Define FOV: front ±30° and rear ±30° around π
        front_fov = (angles > -np.pi/6) & (angles < np.pi/6)
        rear_fov = (angles > 5*np.pi/6) | (angles < -5*np.pi/6)  # wraps around -π/π

        in_fov = front_fov | rear_fov
        angles = angles[in_fov]  # Filter angles corresponding to filtered ranges

        ranges = ranges_full[in_fov]
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time == 0:
            self.last_time = current_time
            self.prev_dist = ranges.copy()  # also filtered
            return
        dt = current_time - self.last_time
        if dt == 0:
            return  # Avoid division by zero

        

        r_dot = np.zeros_like(ranges_full)
        ittc = np.zeros_like(ranges_full)

        r_dot = self.current_vel * np.cos(angles)
        ittc = np.where(r_dot > 0, ranges / r_dot, np.inf)
            
        min_ittc = np.min(ittc)
        min_val_index = np.argmin(ittc)

        self.get_logger().info(f"iTTC: {min_ittc:.3f} s | Relative Velocity: {r_dot[min_val_index]:.3f} m/s | {ranges[min_val_index]}||{np.min(ranges)} || {dt} || {min_val_index}")

        if min_ittc <= self.ittc_threshold:
            self.get_logger().warn("Emergency stop triggered!")
            twist = Twist()
            self.publisher.publish(twist)
            rclpy.shutdown()  # Safer than exit(0)
            return








def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
