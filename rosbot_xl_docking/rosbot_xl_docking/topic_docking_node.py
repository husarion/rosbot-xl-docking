import rclpy
import rclpy.node
import rclpy.qos
from std_msgs.msg import Empty
from rclpy.action import ActionClient
from opennav_docking_msgs.action import DockRobot, UndockRobot

class TopicDocking(rclpy.node.Node):
    def __init__(self):
        super().__init__("topic_docking")

        # Create action clients for the dock and undock actions
        self.dock_client = ActionClient(self, DockRobot, "dock_robot")
        self.undock_client = ActionClient(self, UndockRobot, "undock_robot")

        # Subscribe to /dock and /undock
        self.create_subscription(
            Empty, "dock", self.dock_callback, 10
        )
        self.create_subscription(
            Empty, "undock", self.undock_callback, 10
        )

    def dock_callback(self, msg: Empty):
        goal = DockRobot.Goal()
        goal.dock_id = "home_dock"
        self.dock_client.send_goal_async(goal)

    def undock_callback(self, msg: Empty):
        goal = UndockRobot.Goal()
        goal.dock_type = "simple_charging_dock"
        self.undock_client.send_goal_async(goal)

def main():
    try:
        rclpy.init()
        node = TopicDocking()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
