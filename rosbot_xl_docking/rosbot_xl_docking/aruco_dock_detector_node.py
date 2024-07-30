import rclpy
import rclpy.node
import rclpy.qos
from geometry_msgs.msg import PoseStamped
from aruco_opencv_msgs.msg import ArucoDetection
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class ArUcoDockDetector(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_dock_detector")

        # Get marker_id from parameter server
        self.marker_id = self.declare_parameter(
            "marker_id",
            0, 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="ID of the ArUco marker at the dock"
            )
        )

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
            if marker.marker_id == self.marker_id.value:
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
