"""COCO80 NanoDet CPU 검출기. 입력 header를 보존한 2D 검출만 발행한다."""
import json
import math
import time

from mapping_semantic.nanodet_detector import COCO_CLASSES, NanoDetDetector


def main(args=None):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    from threading import Lock

    class DetectorNode(Node):
        def __init__(self):
            super().__init__('nanodet_detector')
            for key, default in {'model_path': '', 'expected_sha256': '',
                                 'input_topic': '/camera/color/image_raw', 'max_hz': 5.0,
                                 'confidence_threshold': 0.35, 'iou_threshold': 0.6}.items():
                self.declare_parameter(key, default)
            value = lambda key: self.get_parameter(key).value
            hz = value('max_hz')
            if not math.isfinite(hz) or not 0 < hz <= 5:
                raise ValueError('max_hz는 0보다 크고 5 이하여야 합니다')
            self.detector = NanoDetDetector(value('model_path'), value('expected_sha256'),
                                           value('confidence_threshold'), value('iou_threshold'))
            self.bridge = CvBridge()
            self.lock = Lock()
            self.latest = None
            self.received = self.replaced = self.processed = self.failed = 0
            self.last_error = ''
            self.last_inference_sec = self.last_queue_sec = None
            self.last_sensor_age_sec = None
            self.last_header = None
            self.inference_group = MutuallyExclusiveCallbackGroup()
            self.input_group = MutuallyExclusiveCallbackGroup()
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Image, value('input_topic'), self.receive, qos,
                                     callback_group=self.input_group)
            self.publisher = self.create_publisher(Detection2DArray, '/semantic/detections2d', 1)
            self.status_publisher = self.create_publisher(String, '/semantic/detector_status', 1)
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(1.0 / hz, self.infer, clock=self.wall_clock,
                              callback_group=self.inference_group)
            self.create_timer(1.0, self.status, clock=self.wall_clock,
                              callback_group=self.inference_group)
            self.get_logger().info('NanoDet CPU 모델 SHA-256: ' + self.detector.sha256)

        def receive(self, message):
            # 추론 중에도 최신 한 장으로 교체한다. DDS 입력 큐 역시 depth=1.
            with self.lock:
                self.received += 1
                if self.latest is not None:
                    self.replaced += 1
                self.latest = (message, time.monotonic())

        def infer(self):
            with self.lock:
                pending, self.latest = self.latest, None
            if pending is None:
                return
            message, received = pending
            start = time.monotonic()
            self.last_queue_sec = start - received
            try:
                if message.encoding not in ('rgb8', 'bgr8'):
                    raise ValueError('rgb8 또는 bgr8 영상만 지원합니다')
                stamp = message.header.stamp
                if (not message.header.frame_id or stamp.sec < 0
                        or not 0 <= stamp.nanosec < 1_000_000_000):
                    raise ValueError('입력 header frame/stamp가 유효하지 않습니다')
                rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
                predictions = self.detector.infer(rgb)
                result = Detection2DArray()
                result.header = message.header
                for row in predictions:
                    detection = Detection2D()
                    detection.header = message.header
                    # 2D 검출에는 추적·객체 식별을 하지 않으므로 id를 만들지 않는다.
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = COCO_CLASSES[int(row[5])]
                    hypothesis.hypothesis.score = float(row[4])
                    detection.results = [hypothesis]
                    detection.bbox.center.position.x = float((row[0] + row[2]) / 2)
                    detection.bbox.center.position.y = float((row[1] + row[3]) / 2)
                    detection.bbox.center.theta = 0.0
                    detection.bbox.size_x = float(row[2] - row[0])
                    detection.bbox.size_y = float(row[3] - row[1])
                    result.detections.append(detection)
                self.publisher.publish(result)
                self.processed += 1
                self.last_header = {'stamp_sec': stamp.sec, 'stamp_nanosec': stamp.nanosec,
                                    'frame_id': message.header.frame_id}
                # 시뮬레이션 입력에는 use_sim_time=true 설정이 필요하다.
                self.last_sensor_age_sec = (self.get_clock().now().nanoseconds
                                           - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1e9
            except Exception as exc:
                self.failed += 1
                self.last_error = f'{type(exc).__name__}: {exc}'
                self.get_logger().error('NanoDet 프레임 처리 실패: ' + self.last_error)
            finally:
                self.last_inference_sec = time.monotonic() - start

        def status(self):
            with self.lock:
                received, replaced = self.received, self.replaced
                queued = self.latest is not None
            message = String()
            message.data = json.dumps({**self.detector.metadata(),
                                       'received': received, 'replaced': replaced,
                                       'processed': self.processed, 'failed': self.failed,
                                       'queued': queued, 'last_error': self.last_error,
                                       'last_frame': self.last_header,
                                       'last_processing_wall_sec': self.last_inference_sec,
                                       'last_queue_wall_sec': self.last_queue_sec,
                                       'last_sensor_age_sec': self.last_sensor_age_sec},
                                      ensure_ascii=False, allow_nan=False)
            self.status_publisher.publish(message)

    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = DetectorNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
