"""SLAM 시작 전에 실행하는 읽기 전용 진단 수집기."""

import argparse
from collections import Counter
import json
import os
import signal
from pathlib import Path
import sys
import time

from mapping_eval.diagnostics import parse_graph, stamp_ns, transform_record
from mapping_eval.drive_route import require_simulation


def atomic_json(path, document):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('x', encoding='utf-8') as stream:
        json.dump(document, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write('\n')
    temporary.replace(path)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--transform-timeout', type=float, default=0.2)
    parser.add_argument('--confirm-simulation', action='store_true')
    options = parser.parse_args(args)
    import math
    try:
        require_simulation(options.confirm_simulation, os.environ)
        if not math.isfinite(options.transform_timeout) or options.transform_timeout < 0:
            raise ValueError('transform-timeout은 유한한 0 이상이어야 합니다')
        output = options.output.expanduser().resolve()
        output.mkdir(parents=True, exist_ok=False)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import MarkerArray

    streams = {}
    counters = Counter()
    summary = {'schema_version': 1, 'status': 'starting', 'events': [], 'jump_sequences': [],
               'transform_timeout': options.transform_timeout, 'first_scan_stamp_ns': None,
               'first_clock_ns': None, 'last_clock_ns': None, 'errors': [],
               'limits': 'graph/TF 상관은 원인 증명이 아님; graph endpoint ID는 좌표 후보임'}
    node = None
    exit_code = 0

    class Capture(Node):
        def __init__(self):
            super().__init__('mapping_capture', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.sequence = 0
            self.previous_tf = None
            self.previous_tf_stamp = None
            self.latest_graph_seq = None
            self.waiting_jumps = []
            self.events_pub = self.create_publisher(String, '/mapping_capture/events', 10)
            self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', lambda m: self.on_sensor(m, 'scan'), qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', lambda m: self.on_sensor(m, 'odom'), qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, '/pose', lambda m: self.on_sensor(m, 'pose'), 10)
            self.create_subscription(TFMessage, '/tf', lambda m: self.on_tf(m, False), qos_profile_sensor_data)
            static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(TFMessage, '/tf_static', lambda m: self.on_tf(m, True), static_qos)
            self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization', self.on_graph,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.VOLATILE))
            self.service_handles = []
            for name in ('slam_start', 'drive_start', 'capture_end'):
                self.service_handles.append(self.create_service(
                    Trigger, '/mapping_capture/mark_' + name,
                    lambda request, response, event=name: self.mark(event, response)))

        def common(self):
            self.sequence += 1
            return {'schema_version': 1, 'seq': self.sequence,
                    'receive_monotonic_ns': time.monotonic_ns(),
                    'receive_ros_ns': self.get_clock().now().nanoseconds}

        def write(self, name, record):
            streams[name].write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
            streams[name].flush()

        def reject(self, kind, exc):
            counters[kind + '_invalid'] += 1
            if len(summary['errors']) < 20:
                summary['errors'].append({'kind': kind, 'error': str(exc)})

        def on_clock(self, message):
            try:
                stamp = stamp_ns(message.clock)
                counters['clock_received'] += 1
                if summary['first_clock_ns'] is None:
                    summary['first_clock_ns'] = stamp
                if summary['last_clock_ns'] is not None and stamp < summary['last_clock_ns']:
                    counters['clock_backwards'] += 1
                summary['last_clock_ns'] = stamp
            except (ValueError, TypeError) as exc:
                self.reject('clock', exc)

        def on_sensor(self, message, kind):
            try:
                timestamp = stamp_ns(message.header.stamp)
                counters[kind + '_received'] += 1
                if kind == 'scan' and summary['first_scan_stamp_ns'] is None:
                    summary['first_scan_stamp_ns'] = timestamp
                self.write('sensor_stamps', {**self.common(), 'kind': kind,
                                             'topic': '/' + kind,
                                             'header_stamp_ns': timestamp,
                                             'frame_id': message.header.frame_id})
            except (ValueError, TypeError) as exc:
                self.reject(kind, exc)

        def on_tf(self, message, is_static):
            counters['tf_static_received' if is_static else 'tf_received'] += 1
            for item in message.transforms:
                if item.header.frame_id != 'map' or item.child_frame_id != 'odom':
                    continue
                try:
                    timestamp = stamp_ns(item.header.stamp)
                    if self.previous_tf_stamp is not None and timestamp < self.previous_tf_stamp:
                        counters['tf_stamp_backwards'] += 1
                    self.previous_tf_stamp = timestamp
                    record = transform_record(item, self.previous_tf, options.transform_timeout)
                    counters['map_odom_received'] += 1
                    if record is None:
                        counters['tf_unchanged'] += 1
                        continue
                    record.update(self.common())
                    record['source_topic'] = '/tf_static' if is_static else '/tf'
                    record['previous_graph_seq'] = self.latest_graph_seq
                    if record['jump']:
                        summary['jump_sequences'].append(record['seq'])
                        self.waiting_jumps.append(record['seq'])
                    self.write('tf', record)
                    self.previous_tf = record
                    counters['tf_records'] += 1
                except (ValueError, TypeError) as exc:
                    self.reject('tf', exc)

        def on_graph(self, message):
            try:
                record = {**self.common(), **parse_graph(message),
                          'preceding_jump_sequences': list(self.waiting_jumps)}
                self.write('graph', record)
                self.latest_graph_seq = record['seq']
                self.waiting_jumps.clear()
                counters['graph_received'] += 1
            except (ValueError, TypeError) as exc:
                self.reject('graph', exc)

        def mark(self, event, response):
            record = {**self.common(), 'event': event,
                      'first_scan_stamp_ns': summary['first_scan_stamp_ns'],
                      'scan_count': counters['scan_received']}
            self.write('events', record)
            summary['events'].append(record)
            message = String()
            message.data = json.dumps(record, ensure_ascii=False, allow_nan=False)
            self.events_pub.publish(message)
            response.success, response.message = True, message.data
            return response

    try:
        for name in ('events', 'tf', 'graph', 'sensor_stamps'):
            streams[name] = (output / (name + '.jsonl')).open('x', encoding='utf-8')
        rclpy.init(args=[])
        node = Capture()
        atomic_json(output / 'ready.json', {
            **node.common(), 'node_name': 'mapping_capture', 'pid': os.getpid(),
            'ROS_DOMAIN_ID': 12, 'GZ_PARTITION': 'codex',
            'topics': ['/clock', '/scan', '/odom', '/pose', '/tf', '/tf_static',
                       '/slam_toolbox/graph_visualization'],
            'services': ['/mapping_capture/mark_' + event for event in ('slam_start', 'drive_start', 'capture_end')],
            'readiness': 'ROS entities constructed; DDS matching must be verified by orchestrator',
        })
        summary['status'] = 'recording'
        print(f'진단 구독 준비: {output}', flush=True)
        rclpy.spin(node)
        summary['status'] = 'completed'
    except KeyboardInterrupt:
        summary['status'] = 'completed'
    except Exception as exc:
        summary['status'] = 'failed'
        summary['error'] = str(exc)
        exit_code = 1
    finally:
        # 반복 SIGINT가 정리 작업 한 단계를 끊어도 summary 저장을 시도한다.
        for cleanup in ([node.destroy_node] if node is not None else []) + [
                lambda: rclpy.shutdown() if rclpy.ok() else None]:
            try:
                cleanup()
            except (Exception, KeyboardInterrupt) as exc:
                counters['cleanup_interrupt_or_error'] += 1
        for stream in streams.values():
            try:
                stream.close()
            except (Exception, KeyboardInterrupt):
                counters['stream_close_errors'] += 1
        summary['counters'] = dict(counters)
        summary['finished_monotonic_ns'] = time.monotonic_ns()
        summary['warnings'] = [name for name, missing in (
            ('clock_missing', not counters['clock_received']),
            ('map_odom_missing', not counters['map_odom_received']),
            ('graph_missing', not counters['graph_received'])) if missing]
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            atomic_json(output / 'summary.json', summary)
        finally:
            signal.signal(signal.SIGINT, previous_handler)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
