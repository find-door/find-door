"""격리된 Gazebo에서 waypoint를 주행하고 고정 초기 정합으로 궤적을 기록한다."""

import argparse
from collections import deque
import csv
import json
import math
import os
from pathlib import Path
import sys
import time

from mapping_eval.check_interfaces import positive_timeout
from mapping_eval.validation import stamp_ns, validate_odom, validate_scan


def validate_route(document):
    """유한한 odom 평면 좌표만 허용한다."""
    if not isinstance(document, dict) or document.get('frame') != 'odom':
        raise ValueError('route frame은 odom이어야 합니다')
    points = document.get('waypoints')
    if not isinstance(points, list) or not points:
        raise ValueError('waypoints는 비어 있지 않은 좌표 목록이어야 합니다')
    result = []
    for point in points:
        if (not isinstance(point, (list, tuple)) or len(point) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in point)):
            raise ValueError('각 waypoint는 유한한 [x, y]여야 합니다')
        result.append(tuple(float(v) for v in point))
    return result


def require_simulation(confirmed, environment):
    if not confirmed or environment.get('ROS_DOMAIN_ID') != '12':
        raise ValueError('--confirm-simulation과 ROS_DOMAIN_ID=12가 필요합니다')
    if environment.get('GZ_PARTITION') != 'codex':
        raise ValueError('GZ_PARTITION=codex가 필요합니다')


def bounded_speed(value):
    speed = positive_timeout(value)
    if speed > 0.5:
        raise argparse.ArgumentTypeError('speed 상한은 0.5 m/s입니다')
    return speed


def quaternion_yaw(q):
    values = [q.x, q.y, q.z, q.w]
    norm = math.sqrt(sum(value * value for value in values))
    if not all(math.isfinite(value) for value in values) or norm < 1e-9:
        raise ValueError('유효하지 않은 quaternion')
    x, y, z, w = [value / norm for value in values]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def inverse_se2(transform):
    x, y, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, -yaw)


def planar_transform(transform):
    if not all(math.isfinite(value) for value in
               (transform.translation.x, transform.translation.y)):
        raise ValueError('TF 위치가 유한하지 않음')
    return (transform.translation.x, transform.translation.y,
            quaternion_yaw(transform.rotation))


def transform_point(transform, point):
    x, y, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * point[0] - s * point[1],
            y + s * point[0] + c * point[1])


def body_command(position, yaw, waypoint, speed, tolerance=0.08):
    dx, dy = waypoint[0] - position[0], waypoint[1] - position[1]
    distance = math.hypot(dx, dy)
    if distance <= tolerance:
        return (0.0, 0.0, 0.0), True
    # 목표 부근 감속, 몸체 yaw와 관계없이 world 좌표 waypoint로 이동.
    gain = min(speed, distance) / distance
    c, s = math.cos(yaw), math.sin(yaw)
    yaw_error = math.atan2(math.sin(-yaw), math.cos(-yaw))
    return ((c * dx + s * dy) * gain,
            (-s * dx + c * dy) * gain,
            max(-0.3, min(0.3, yaw_error))), False


def freshness_problem(now, clock_advanced_at, odom_at, scan_at):
    for name, received in [('/clock 진행', clock_advanced_at),
                           ('/odom', odom_at), ('/scan', scan_at)]:
        if received is None or now - received > 2.0:
            return f'{name} 미수신 또는 벽시계 2초 이상 정지'
    return None


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=positive_timeout, default=240.0)
    parser.add_argument('--speed', type=bounded_speed, default=0.35)
    parser.add_argument('--confirm-simulation', action='store_true')
    options = parser.parse_args(args)
    import yaml
    try:
        require_simulation(options.confirm_simulation, os.environ)
        with open(options.route, encoding='utf-8') as stream:
            waypoints = validate_route(yaml.safe_load(stream))
    except (ValueError, OSError, yaml.YAMLError) as exc:
        parser.error(str(exc))

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    output = Path(options.output)
    output.mkdir(parents=True, exist_ok=True)
    # 실행 간 궤적/보고서를 실수로 혼합하거나 덮어쓰지 않는다.
    if any((output / name).exists() for name in ('trajectory.csv', 'route_result.json')):
        parser.error('output에 기존 trajectory.csv 또는 route_result.json이 있습니다')

    class RouteDriver(Node):
        def __init__(self):
            super().__init__('mapping_route_driver', parameter_overrides=[
                Parameter('use_sim_time', value=True)])
            self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.clock_ns = None
            self.clock_advanced_at = None
            self.odom = None
            self.odom_at = None
            self.scan = None
            self.scan_at = None
            self.fault = None
            self.pending = deque()
            self.last_odom_ns = None
            self.anchor = None
            self.anchor_stamp_ns = None
            self.initial_pose = None
            self.missing_tf_count = 0
            self.samples = []
            self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        def on_clock(self, message):
            current = stamp_ns(message.clock)
            if self.clock_ns is not None:
                if current < self.clock_ns:
                    self.fault = '/clock 역행'
                elif current > self.clock_ns:
                    self.clock_advanced_at = time.monotonic()
            self.clock_ns = current

        def on_odom(self, message):
            problems = validate_odom(message)
            try:
                quaternion_yaw(message.pose.pose.orientation)
                if not all(math.isfinite(value) for value in
                           (message.pose.pose.position.x, message.pose.pose.position.y)):
                    problems.append('odom 위치가 유한하지 않음')
            except ValueError as exc:
                problems.append(str(exc))
            if problems:
                self.fault = '; '.join(problems)
                return
            current = stamp_ns(message.header.stamp)
            if self.last_odom_ns is not None and current < self.last_odom_ns:
                self.fault = '/odom 타임스탬프 역행'
                return
            if current == self.last_odom_ns:
                return
            self.last_odom_ns = current
            self.odom, self.odom_at = message, time.monotonic()
            self.pending.append((message, self.odom_at))

        def on_scan(self, message):
            problems = validate_scan(message)
            if problems:
                self.fault = '; '.join(problems)
                return
            valid = [v for v in message.ranges
                     if math.isfinite(v) and message.range_min <= v <= message.range_max]
            if min(valid) < 0.25:
                self.fault = 'LiDAR 장애물 거리 < 0.25 m'
            self.scan, self.scan_at = message, time.monotonic()

        def publish(self, command=(0.0, 0.0, 0.0)):
            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = command
            self.publisher.publish(message)

        def safety_problem(self):
            if self.fault:
                return self.fault
            issue = freshness_problem(time.monotonic(), self.clock_advanced_at,
                                      self.odom_at, self.scan_at)
            if issue:
                return issue
            if self.count_publishers('/clock') != 1:
                return '/clock 발행자는 정확히 하나여야 함'
            for name, message in [('/odom', self.odom), ('/scan', self.scan)]:
                age = self.clock_ns - stamp_ns(message.header.stamp)
                if not -100_000_000 <= age <= 2_000_000_000:
                    return f'{name} 시뮬 시간 기준 오래되거나 미래인 관측'
            return None

        def record(self):
            now = time.monotonic()
            while self.pending:
                message, received = self.pending[0]
                when = Time.from_msg(message.header.stamp)
                try:
                    if self.anchor is None:
                        tf = self.buffer.lookup_transform('map', 'odom', when).transform
                        self.anchor = inverse_se2(planar_transform(tf))
                        self.anchor_stamp_ns = stamp_ns(message.header.stamp)
                        self.initial_pose = [message.pose.pose.position.x,
                                             message.pose.pose.position.y,
                                             quaternion_yaw(message.pose.pose.orientation)]
                    tf = self.buffer.lookup_transform('map', 'base_footprint', when).transform
                    measured = planar_transform(tf)
                except TransformException:
                    if now - received < 1.0:
                        break
                    self.pending.popleft()
                    self.missing_tf_count += 1
                    continue
                except ValueError as exc:
                    self.fault = str(exc)
                    self.missing_tf_count += len(self.pending)
                    self.pending.clear()
                    return
                self.pending.popleft()
                map_position = measured[:2]
                estimated = transform_point(self.anchor, map_position)
                actual = (message.pose.pose.position.x, message.pose.pose.position.y)
                self.samples.append({
                    'stamp_ns': stamp_ns(message.header.stamp),
                    'odom_x': actual[0], 'odom_y': actual[1],
                    'map_x': map_position[0], 'map_y': map_position[1],
                    'estimated_world_x': estimated[0], 'estimated_world_y': estimated[1],
                    'error_m': math.hypot(estimated[0] - actual[0], estimated[1] - actual[1]),
                })

    rclpy.init(args=[])
    node = None
    started = time.monotonic()
    completed = 0
    stop_cause = '초기화 실패'
    success = False
    ready_once = False
    settling_at = None
    last_step = started
    try:
        node = RouteDriver()
        while rclpy.ok():
            if time.monotonic() - started >= options.timeout:
                stop_cause = '벽시계 제한 시간 초과'
                break
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - last_step < 0.05:
                continue
            last_step = time.monotonic()
            node.record()
            problem = node.safety_problem()
            if problem:
                node.publish()
                if node.fault or ready_once or time.monotonic() - started > 15.0:
                    stop_cause = problem
                    break
                continue
            if node.anchor is None or not node.samples:
                node.publish()
                if time.monotonic() - started > 15.0:
                    stop_cause = '초기 odom 시각의 SLAM TF 확보 실패'
                    break
                continue
            ready_once = True
            if settling_at is not None:
                node.publish()
                if node.clock_ns - settling_at >= 2_000_000_000:
                    success, stop_cause = True, '모든 waypoint 도달 및 시뮬 시간 2초 정지'
                    break
                continue
            pose = node.odom.pose.pose
            command, reached = body_command(
                (pose.position.x, pose.position.y), quaternion_yaw(pose.orientation),
                waypoints[completed], options.speed)
            if reached:
                completed += 1
                if completed == len(waypoints):
                    settling_at = node.clock_ns
            node.publish(command)
        else:
            stop_cause = 'ROS context 종료'
    except KeyboardInterrupt:
        stop_cause = '사용자 중단'
    except Exception as exc:
        stop_cause = f'{type(exc).__name__}: {exc}'
    finally:
        if node is not None:
            # 성공·실패·중단 모두 정지 명령. SIGKILL/전원 단절은 보장하지 않는다.
            for _ in range(3):
                try:
                    node.publish()
                    time.sleep(0.05)
                except Exception:
                    break
            node.record()
            node.missing_tf_count += len(node.pending)
            columns = ['stamp_ns', 'odom_x', 'odom_y', 'map_x', 'map_y',
                       'estimated_world_x', 'estimated_world_y', 'error_m']
            with (output / 'trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=columns)
                writer.writeheader()
                writer.writerows(node.samples)
            errors = [sample['error_m'] for sample in node.samples]
            trace_total = len(node.samples) + node.missing_tf_count
            trace_fraction = len(node.samples) / trace_total if trace_total else 0.0
            trace_recent = bool(node.samples) and node.clock_ns - node.samples[-1]['stamp_ns'] <= 1_000_000_000
            trajectory_valid = trace_fraction >= 0.95 and trace_recent
            result = {
                'success': success, 'stop_cause': stop_cause,
                'elapsed_wall_sec': time.monotonic() - started,
                'route': str(Path(options.route).resolve()), 'frame': 'odom',
                'waypoints_total': len(waypoints), 'waypoints_completed': completed,
                'speed_mps': options.speed, 'waypoint_tolerance_m': 0.08,
                'initial_map_to_world': node.anchor,
                'map_to_world': node.anchor,
                'initial_transform_stamp_ns': node.anchor_stamp_ns,
                'initial_pose': node.initial_pose,
                'alignment': 'initial map<-odom inverse, fixed; no final fitting',
                'ground_truth': 'Gazebo odom position (world-origin simulation only)',
                'trajectory_valid': trajectory_valid, 'trace_fraction': trace_fraction,
                'trace_count': len(node.samples), 'missing_tf_count': node.missing_tf_count,
                'position_rmse_m': math.sqrt(sum(e * e for e in errors) / len(errors)) if errors and trajectory_valid else None,
                'position_max_error_m': max(errors) if errors and trajectory_valid else None,
            }
            (output / 'route_result.json').write_text(
                json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                encoding='utf-8')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'{"통과" if success else "실패"}: {stop_cause} ({completed}/{len(waypoints)})')
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
