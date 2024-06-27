import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped


class DockPosePublisher(Node):
    def __init__(self):
        super().__init__("dock_pose_publisher")
        self.get_logger().info("AAAAE")
        self.subscription = self.create_subscription(
            AprilTagDetectionArray, "apriltag_detections", self.detection_callback, 10
        )
        self.publisher = self.create_publisher(PoseStamped, "detected_dock_pose", 10)
        self.use_first_detection = (
            # self.declare_parameter("use_first_detection", True)
            self.declare_parameter("use_first_detection", False)
            .get_parameter_value()
            .bool_value
        )
        self.dock_tag_family = (
            self.declare_parameter("dock_tag_family", "tag36h11")
            .get_parameter_value()
            .string_value
        )
        self.dock_tag_id = (
            self.declare_parameter("dock_tag_id", 30)
            .get_parameter_value()
            .integer_value
        )

    def detection_callback(self, msg):
        pose = PoseStamped()
        for detection in msg.detections:
            if not self.use_first_detection:
                if (
                    # detection.family == self.dock_tag_family
                    # and detection.id == self.dock_tag_id
                    #
                    detection.id == self.dock_tag_id
                ):
                    pose.header = msg.header
                    pose.pose = detection.pose.pose.pose
                    self.publisher.publish(pose)
                    return
            else:
                pose.header = msg.header
                pose.pose = msg.detections[0].pose.pose.pose
                self.publisher.publish(pose)
                return


def main(args=None):
    rclpy.init(args=args)
    dock_pose_publisher = DockPosePublisher()
    rclpy.spin(dock_pose_publisher)
    dock_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
