"""카메라 토픽 프레임을 벽시계 시각 이름의 JPEG로 저장한다 (10 Hz 이하)."""
import sys, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from PIL import Image as PImage

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)

class Rec(Node):
    def __init__(self):
        super().__init__('demo_camera_recorder')
        self.last = 0.0
        self.create_subscription(Image, '/camera/color/image_raw', self.cb, qos_profile_sensor_data)
    def cb(self, m):
        now = time.time()
        if now - self.last < 0.095:
            return
        self.last = now
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        if m.encoding == 'bgr8':
            a = a[:, :, ::-1]
        PImage.fromarray(a).save(out / f'{time.time_ns()}.jpg', quality=92)

rclpy.init(); n = Rec()
try:
    rclpy.spin(n)
except (KeyboardInterrupt, Exception):
    pass
