import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class camera(Node):
    def __init__(self):
        super().__init__("feed")
        self.feed = self.create_subscription(
            Image, "/bcr_bot/stereo_camera/left/image_raw", self.feed_conversion_callback, 10)
        self.bridge = CvBridge()

    def feed_conversion_callback(self,Image):
        cv_image = self.bridge.imgmsg_to_cv2(Image, desired_encoding='bgr8')

        cv.imshow("Camera", cv_image)
        cv.waitKey(1)



def main(args = None):
    rclpy.init(args=args)
    node = camera()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
