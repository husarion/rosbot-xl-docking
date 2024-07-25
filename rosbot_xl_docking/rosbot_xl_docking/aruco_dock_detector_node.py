import rclpy
import rclpy.node
import rclpy.qos
import tf2_ros
from geometry_msgs.msg import PoseStamped
from aruco_opencv_msgs.msg import ArucoDetection

class ArUcoDockDetector(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_dock_detector")

        # Subscribe to /aruco_detections
        self.create_subscription(
            ArucoDetection, "aruco_detections", self.detection_callback, 10
        )

        # Publish the pose of the detected dock
        self.pub = self.create_publisher(
            PoseStamped, "detected_dock_pose", 10
        )

    def detection_callback(self, msg: ArucoDetection):
        for marker in msg.markers:
            if marker.marker_id == 4:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose = marker.pose
                self.pub.publish(pose)


def main():
    try:
        rclpy.init()
        node = ArUcoDockDetector()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
