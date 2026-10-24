"""시뮬레이션 입력과 선택적 SLAM 출력을 제한 시간 안에 확인한다."""

import argparse
from collections import deque
import math
import sys
import time

from mapping_eval.validation import (
    stamp_ns, validate_map, validate_odom, validate_odom_age, validate_scan,
)


def positive_timeout(value):
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('timeout은 양수여야 합니다') from exc
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('timeout은 유한한 양수여야 합니다')
    return number


def main(args=None):
    parser = argparse.ArgumentParser(description='시뮬레이션 센서·TF 인터페이스 검사')
    parser.add_argument('--timeout', type=positive_timeout, default=20.0,
                        help='벽시계 기준 제한 시간(초), 기본값 20')
    parser.add_argument('--require-map', action='store_true',
                        help='/map과 map → odom TF도 검사')
    options = parser.parse_args(args)

    # --help와 순수 검증 테스트는 ROS 환경 없이도 사용할 수 있다.
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    class InterfaceCheck(Node):
        def __init__(self):
            super().__init__(
                'mapping_interface_check',
                parameter_overrides=[Parameter('use_sim_time', value=True)],
            )
            self.scans = deque(maxlen=10)
            self.scan_count = 0
            self.odom = None
            self.occupancy_map = None
            self.first_clock = None
            self.latest_clock = None
            self.clock_advanced = False
            self.clock_reversed = False
            self.clock_changed_at = 0.0
            self.scan_at = 0.0
            self.odom_at = 0.0
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            sensor_qos = QoSProfile(
                depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.create_subscription(Clock, '/clock', self.on_clock, sensor_qos)
            self.create_subscription(LaserScan, '/scan', self.on_scan, sensor_qos)
            self.create_subscription(Odometry, '/odom', self.on_odom, sensor_qos)
            if options.require_map:
                self.create_subscription(
                    OccupancyGrid, '/map', self.on_map,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL),
                )

        def on_clock(self, message):
            value = stamp_ns(message.clock)
            if self.latest_clock is not None and value < self.latest_clock:
                # TF 버퍼에 이전 실행의 변환이 남을 수 있으므로 이 실행은
                # 통과시키지 않는다. 시뮬 reset 뒤 검사 노드를 다시 실행한다.
                self.clock_reversed = True
                self.clock_advanced = False
                self.clock_changed_at = 0.0
                self.scans.clear()
                self.scan_count = 0
                self.odom = None
                self.occupancy_map = None
                self.scan_at = 0.0
                self.odom_at = 0.0
            if self.first_clock is None:
                self.first_clock = value
            if self.latest_clock is not None and value > self.latest_clock:
                self.clock_advanced = True
                self.clock_changed_at = time.monotonic()
            self.latest_clock = value

        def on_scan(self, message):
            self.scans.append(message)
            self.scan_count += 1
            self.scan_at = time.monotonic()

        def on_odom(self, message):
            self.odom = message
            self.odom_at = time.monotonic()

        def on_map(self, message):
            self.occupancy_map = message

        def problems(self):
            errors = []
            now = time.monotonic()
            publishers = self.count_publishers('/clock')
            if self.clock_reversed:
                errors.append('/clock이 역행했습니다. 시뮬 reset 후 검사를 다시 실행하세요')
            if publishers != 1:
                errors.append(f'/clock 발행자 수={publishers} (기대값 1)')
            if not self.clock_advanced or now - self.clock_changed_at > 2.0:
                errors.append('/clock 진행이 확인되지 않거나 2초 이상 멈췄습니다')
            if self.scan_count < 2 or now - self.scan_at > 2.0:
                errors.append('/scan을 최근에 2회 이상 수신해야 합니다')
            if not self.scans:
                errors.append('/scan 미수신')
            else:
                errors.extend(validate_scan(self.scans[-1]))
            if self.odom is None or now - self.odom_at > 2.0:
                errors.append('/odom 미수신 또는 2초 이상 갱신되지 않았습니다')
            else:
                errors.extend(validate_odom(self.odom))
                errors.extend(validate_odom_age(self.odom, self.latest_clock))
            if options.require_map:
                if self.occupancy_map is None:
                    errors.append('/map 미수신 (TRANSIENT_LOCAL + RELIABLE 구독)')
                else:
                    errors.extend(validate_map(self.occupancy_map))

            # 가장 최신 scan의 TF가 아직 도착하지 않았을 수 있어 짧은 큐에서
            # 최근 유효 관측을 찾는다. 시간 0(최신 TF 조회)은 사용하지 않는다.
            tf_ready = False
            for scan in reversed(self.scans):
                if validate_scan(scan):
                    continue
                if (self.latest_clock is None
                        or not 0 <= self.latest_clock - stamp_ns(scan.header.stamp)
                        <= 1_000_000_000):
                    continue
                when = Time.from_msg(scan.header.stamp)
                pairs = [('odom', 'base_footprint'),
                         ('base_footprint', 'lidar_sim_frame')]
                if options.require_map:
                    pairs.append(('map', 'odom'))
                try:
                    for target, source in pairs:
                        self.buffer.lookup_transform(target, source, when)
                    tf_ready = True
                    break
                except TransformException:
                    continue
            if not tf_ready:
                errors.append('최근 scan 시각의 필수 TF 연결을 조회할 수 없습니다')
            return errors

    rclpy.init(args=[])
    node = None
    try:
        node = InterfaceCheck()
        deadline = time.monotonic() + options.timeout
        errors = ['검사를 시작하지 못했습니다']
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            errors = node.problems()
            if not errors:
                detail = ', /map 및 map → odom' if options.require_map else ''
                print(f'통과: /clock, /scan, /odom, 센서 시각 TF{detail}')
                return 0
        print('실패: 인터페이스 검사 제한 시간 초과', file=sys.stderr)
        for error in errors:
            print(f'- {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('실패: 사용자가 검사를 중단했습니다', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
