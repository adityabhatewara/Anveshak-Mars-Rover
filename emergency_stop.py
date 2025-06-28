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
            Odometry, "/bcr_bot/Odom", self.Odom_callback, qos_profile)
        
        # inside __init__()
        self.angle_start = -np.pi / 6 # -30 degrees
        self.angle_end = np.pi / 6     # +30 degrees


        self.prev_dist = None
        self.last_time = 0.0
        self.ittc_threshold = 0.5
        self.current_vel = 0.0

    def Odom_callback(self,msg):
        self.current_vel = msg.twist.twist.linear.x
        
    

    def vel_callback(self, msg):
        
        #self.get_logger().info(ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        # index_max = 
        # angles = angle_min + np.arange(len(ranges)) * angle_increment
        # fov_limit = np.deg2rad(20)
        # in_fov = (angles > -fov_limit) & (angles < fov_limit)


        # Full preprocessed LIDAR array
        # ranges_full = np.array(msg.ranges)
        # ranges_full = np.where(np.isfinite(ranges_full), ranges_full, 10.0)

        # # Compute angles per index
        # angle_min = msg.angle_min
        # angle_increment = msg.angle_increment
        # angles = angle_min + np.arange(len(ranges_full)) * angle_increment

        # # Normalize angles to [-π, π]
        # angles = (angles + np.pi) % (2 * np.pi) - np.pi

        # # Define FOV: front ±30° and rear ±30° around π
        # front_fov = (angles > -np.pi/6) & (angles < np.pi/6)
        # rear_fov = (angles > 5*np.pi/6) | (angles < -5*np.pi/6)  # wraps around -π/π

        # in_fov = front_fov | rear_fov
        # ranges = ranges_full[in_fov]

        ranges = np.array(msg.ranges)
        # self.get_logger().info(ranges)
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time == 0:
            self.last_time = current_time
            self.prev_dist = ranges.copy()  # also filtered
            return

        dt = current_time - self.last_time
        if dt == 0:
            return  # Avoid division by zero

        ranges = np.where(np.isfinite(ranges), ranges, 10.0)
        
        self.prev_dist = np.where(np.isfinite(self.prev_dist), self.prev_dist, 10.0)

        # r_dot = (self.prev_dist - ranges) / dt
        # r_dot = np.where(r_dot < 0.001, 0.0, r_dot)
        # r_dot = np.array(self.current_vel)

        for i in range(len(ranges)):
            angle = angle_increment * i
            if np.cos(angle) * self.current_vel > 0:
                r_dot[i] = self.current_vel * np.cos(angle)

        ittc = np.full_like(ranges, np.inf)

        for i in range(len(ranges)):
            if r_dot[i] > 0:
                ittc[i] = ranges[i] / r_dot[i]

        min_ittc = np.min(ittc)
        min_val_index = np.argmin(ittc)

        self.get_logger().info(f"iTTC: {min_ittc:.3f} s | Relative Velocity: {r_dot[min_val_index]:.3f} m/s | {r_dot[min_val_index]}, {ranges[min_val_index]}||{np.min(ranges)} || {dt} || {min_val_index}")

        if min_ittc <= self.ittc_threshold and ranges[min_val_index] < 1.5:
            self.get_logger().warn("Emergency stop triggered!")
            twist = Twist()
            self.publisher.publish(twist)
            rclpy.shutdown()  # Safer than exit(0)
            return

        self.prev_dist = ranges.copy()
        self.last_time = current_time



def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
