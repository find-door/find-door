"""주행 명령 없이 replay odom과 새 SLAM TF를 관측한다."""

from collections import deque
import csv
import json
import math
from pathlib import Path
import time

from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from mapping_eval.diagnostics import parse_graph
from mapping_eval.drive_route import inverse_se2, planar_transform, transform_point
from mapping_eval.bag_filter import replay_timing_issues
from mapping_eval.trajectory_metrics import COLUMNS
from mapping_eval.validation import stamp_ns, validate_map, validate_odom


class ReplayObserver(Node):
    """첫 유효 map←odom의 역변환을 고정하고 이후 관측에 적용한다."""

    def __init__(self):
        super().__init__('mapping_replay_observer', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = deque()
        self.samples = []
        self.anchor = None
        self.anchor_stamp_ns = None
        self.clock_ns = None
        self.last_odom_ns = None
        self.received_odom_count = 0
        self.missing_tf_count = 0
        self.map = None
        self.fault = None
        self.graph_records = []
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # slam_toolbox 전용 토픽이라 Cartographer 재생에서는 비어 있다.
        self.create_subscription(
            MarkerArray, '/slam_toolbox/graph_visualization', self.on_graph,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))

    def on_graph(self, message):
        self.graph_records.append({'receive_ros_ns': self.clock_ns or 0,
                                   'nodes': parse_graph(message)['nodes']})

    def on_clock(self, message):
        current = stamp_ns(message.clock)
        if self.clock_ns is not None and current < self.clock_ns:
            self.fault = 'replay /clock이 역행했습니다'
        self.clock_ns = current

    def on_odom(self, message):
        issues = validate_odom(message)
        position = message.pose.pose.position
        if not all(math.isfinite(value) for value in (position.x, position.y)):
            issues.append('odom 위치가 유한하지 않습니다')
        if issues:
            self.fault = '; '.join(issues)
            return
        current = stamp_ns(message.header.stamp)
        if self.last_odom_ns is not None and current <= self.last_odom_ns:
            self.fault = 'replay odom timestamp가 엄격히 증가하지 않습니다'
            return
        self.last_odom_ns = current
        self.received_odom_count += 1
        self.pending.append((message, time.monotonic()))

    def on_map(self, message):
        self.map = message

    def record(self):
        while self.pending:
            message, received = self.pending[0]
            when = Time.from_msg(message.header.stamp)
            try:
                relation = self.buffer.lookup_transform('map', 'odom', when).transform
                measured = planar_transform(
                    self.buffer.lookup_transform('map', 'base_footprint', when).transform)
                relation = planar_transform(relation)
            except TransformException:
                if time.monotonic() - received < 2.0:
                    break
                self.pending.popleft()
                self.missing_tf_count += 1
                continue
            except ValueError as exc:
                self.fault = str(exc)
                return
            if self.anchor is None:
                self.anchor = inverse_se2(relation)
                self.anchor_stamp_ns = stamp_ns(message.header.stamp)
            self.pending.popleft()
            estimated = transform_point(self.anchor, measured[:2])
            actual = message.pose.pose.position
            self.samples.append({
                'stamp_ns': stamp_ns(message.header.stamp),
                'odom_x': actual.x, 'odom_y': actual.y,
                'map_x': measured[0], 'map_y': measured[1],
                'estimated_world_x': estimated[0], 'estimated_world_y': estimated[1],
                'error_m': math.hypot(estimated[0] - actual.x, estimated[1] - actual.y),
            })

    def finish(self, folder, expected_odom_count, expected_last_odom_ns):
        """caller가 playback 종료 후 2초 이상 spin/drain한 다음 호출한다."""
        folder = Path(folder)
        self.record()
        self.missing_tf_count += len(self.pending)
        self.pending.clear()
        with (folder / 'trajectory.csv').open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=COLUMNS)
            writer.writeheader()
            writer.writerows(self.samples)
        if self.graph_records:
            with (folder / 'graph.jsonl').open('x', encoding='utf-8') as stream:
                stream.writelines(json.dumps(record) + '\n' for record in self.graph_records)
        fraction = len(self.samples) / expected_odom_count if expected_odom_count else 0.0
        received_fraction = (self.received_odom_count / expected_odom_count
                             if expected_odom_count else 0.0)
        last_sample = self.samples[-1]['stamp_ns'] if self.samples else None
        last_map = stamp_ns(self.map.header.stamp) if self.map is not None else None
        timing = replay_timing_issues(expected_last_odom_ns, last_sample, self.clock_ns, last_map)
        recent = (last_sample is not None and expected_last_odom_ns is not None
                  and 0 <= expected_last_odom_ns - last_sample <= 1_000_000_000)
        map_issues = validate_map(self.map) if self.map is not None else ['/map 미수신']
        map_issues += timing['map']
        invalid_reasons = list(timing['trajectory'])
        if fraction < 0.95:
            invalid_reasons.append('원본 bag 대비 궤적 표본 비율이 95% 미만입니다')
        if received_fraction < 0.99:
            invalid_reasons.append('원본 bag 대비 odom 수신 비율이 99% 미만입니다')
        if self.anchor is None:
            invalid_reasons.append('초기 map_to_world 변환이 없습니다')
        if self.fault:
            invalid_reasons.append(self.fault)
        report = {
            'initial_map_to_world': self.anchor,
            'initial_transform_stamp_ns': self.anchor_stamp_ns,
            'alignment': 'inverse initial map<-odom, fixed; no final fitting',
            'limitations': 'Gazebo odom also feeds SLAM; not independent ATE',
            'expected_odom_count': expected_odom_count,
            'expected_last_odom_ns': expected_last_odom_ns,
            'received_odom_count': self.received_odom_count,
            'received_odom_fraction': received_fraction,
            'trace_count': len(self.samples), 'trace_fraction': fraction,
            'missing_tf_count': self.missing_tf_count,
            'trace_recent': recent, 'fault': self.fault, 'map_issues': map_issues,
            'trajectory_invalid_reasons': invalid_reasons,
            'last_sample_stamp_ns': last_sample,
            'last_clock_ns': self.clock_ns,
            'last_map_stamp_ns': last_map,
            'map_at_eof_policy': 'last available map at EOF; wall-time drain does not force map update',
            'trajectory_valid': not invalid_reasons,
        }
        (folder / 'replay_observation.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
            encoding='utf-8')
        return report
