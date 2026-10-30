"""TF map → base_footprint(SLAM 추정 위치)를 쌓아 nav_msgs/Path로 발행한다."""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class SlamPath(Node):
    def __init__(self):
        super().__init__('demo_slam_path',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Path, '/demo/slam_path', 10)
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.create_timer(0.2, self.tick)

    def tick(self):
        try:
            t = self.buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return
        pose = PoseStamped()
        pose.header = t.header
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.orientation = t.transform.rotation
        self.path.poses.append(pose)
        self.path.header.stamp = t.header.stamp
        self.publisher.publish(self.path)


rclpy.init()
node = SlamPath()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, Exception):
    pass
