"""동일 bag 센서 입력으로 SLAM을 재실행한다. 원래 /clock만 재생한다."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from mapping_eval.bag_filter import filter_bag, validate_output
from mapping_eval.drive_route import require_simulation
from mapping_eval.graph_motion import analyze_graph
from mapping_eval.map_metrics import evaluate_map
from mapping_eval.run_benchmark import digest, positive, stop_process
from mapping_eval.trajectory_metrics import analyze_trajectory


def playback_command(bag, rate=1.0, qos_overrides=None):
    """clock 합성 없이 paused 상태에서 subscriber 연결을 기다린다."""
    command = ['ros2', 'bag', 'play', str(bag), '--start-paused',
            '--disable-keyboard-controls', '--rate', str(rate),
            '--wait-for-all-acked', '2000']
    if qos_overrides is not None:
        command.extend(['--qos-profile-overrides-path', str(qos_overrides)])
    return command


def cleanup_children(children, handles, stop=stop_process):
    """하나의 종료 실패가 다른 자식 정리를 건너뛰게 하지 않는다."""
    problems = []
    for child in reversed(children):
        try:
            if stop(child) is False:
                problems.append(f'프로세스 그룹 {child.pid}이 종료되지 않았습니다')
        except Exception as exc:
            problems.append(str(exc))
    for handle in handles:
        try:
            handle.close()
        except Exception as exc:
            problems.append(str(exc))
    return problems


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--params-file', required=True, type=Path)
    parser.add_argument('--truth', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='새 결과 디렉토리')
    parser.add_argument('--slam-launch', default='mapping.launch.py',
                        help='mapping_slam 패키지 내 launch 파일 이름')
    parser.add_argument('--slam-node', default='/slam_toolbox',
                        help='실효 파라미터 dump 대상 노드')
    parser.add_argument('--rate', type=positive, default=1.0)
    parser.add_argument('--confirm-simulation', action='store_true')
    options = parser.parse_args(args)
    try:
        isolation = require_simulation(options.confirm_simulation, os.environ)
        source, output = validate_output(options.bag, options.output)
        params, truth = options.params_file.resolve(), options.truth.resolve()
        if not params.is_file() or not truth.is_file():
            raise ValueError('params-file과 truth는 기존 파일이어야 합니다')
        if (Path(options.slam_launch).name != options.slam_launch
                or not options.slam_launch.endswith('.launch.py')):
            raise ValueError('slam-launch는 경로 없이 *.launch.py 파일 이름이어야 합니다')
    except ValueError as exc:
        parser.error(str(exc))
    output.mkdir(parents=True, exist_ok=False)
    inputs = output / 'inputs'
    inputs.mkdir()
    children, handles = [], []
    node = None
    ros_initialized = False
    observer_finished = False
    filter_report = None
    status = {'schema_version': 1, 'status': 'running', 'source_bag': str(source),
              'slam_launch': options.slam_launch, 'slam_node': options.slam_node,
              'rate': options.rate, **isolation,
              'clock_policy': 'replay original /clock; never ros2 bag play --clock',
              'alignment': 'initial map_to_world fixed; no fitting',
              'map_at_eof_policy': 'last available map at EOF; 2.5s wall drain does not force map update',
              'limitations': 'same inputs, not bitwise deterministic SLAM; '
                             'Gazebo odom also feeds SLAM, not independent ATE'}
    env = os.environ.copy()
    env['ROS_LOG_DIR'] = str(output / 'roslogs')

    def save_manifest():
        (output / 'replay_manifest.json').write_text(
            json.dumps(status, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
            encoding='utf-8')

    def start(command, name):
        handle = (output / (name + '.log')).open('w')
        handles.append(handle)
        child = subprocess.Popen(command, env=env, stdout=handle,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        status.setdefault('commands', {})[name] = command
        return child

    def spin():
        rclpy.spin_once(node, timeout_sec=0.01)
        node.record()
        if node.fault:
            raise RuntimeError(node.fault)

    def wait_child(child, timeout, label):
        deadline = time.monotonic() + timeout
        while child.poll() is None:
            if time.monotonic() > deadline:
                raise RuntimeError(label + ' 벽시계 제한 시간 초과')
            spin()
        if child.returncode != 0:
            raise RuntimeError(f'{label} 종료 코드 {child.returncode}')

    try:
        from ament_index_python.packages import get_package_share_directory
        qos_source = Path(get_package_share_directory('mapping_eval')) / 'config/replay_input_qos.yaml'
        qos = inputs / 'replay_input_qos.yaml'
        qos.write_bytes(qos_source.read_bytes())
        status['playback_qos_policy'] = 'explicit filtered-input profile; tf_static transient_local; original metadata unchanged'
        copied_params = inputs / ('slam' + (params.suffix or '.yaml'))
        for name, path in [(copied_params.name, params), ('truth.yaml', truth)]:
            (inputs / name).write_bytes(path.read_bytes())
        status['input_sha256'] = {p.name: digest(p) for p in inputs.iterdir()}
        status['code_sha256'] = {
            p.name: digest(p) for p in Path(__file__).resolve().parent.glob('*.py')}
        save_manifest()
        import rclpy
        from rosbag2_interfaces.srv import Resume
        from mapping_eval.replay_observer import ReplayObserver
        rclpy.init(args=[])
        ros_initialized = True
        node = ReplayObserver()
        probe_until = time.monotonic() + 3.0
        while time.monotonic() < probe_until:
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(node.count_publishers(topic) for topic in ('/clock', '/odom', '/map', '/tf')):
                raise RuntimeError('도메인 12에 기존 ROS 발행자가 있습니다. 종료하지 않고 거부합니다')
        filter_report = filter_bag(source, output / 'filtered')
        status['filter_manifest'] = str(output / 'filtered/filter_manifest.json')
        status['retained_record_payload_sha256'] = filter_report['retained_record_payload_sha256']
        slam = start(['ros2', 'launch', 'mapping_slam', options.slam_launch,
                      'use_sim_time:=true', 'params_file:=' + str(copied_params)], 'slam')
        player = start(playback_command(filter_report['bag'], options.rate, qos), 'playback')
        resume = node.create_client(Resume, '/rosbag2_player/resume')
        startup_deadline = time.monotonic() + 30.0
        ready_at = None
        while True:
            if slam.poll() is not None or player.poll() is not None:
                raise RuntimeError('SLAM 또는 paused bag player 시작 실패')
            if time.monotonic() > startup_deadline:
                raise RuntimeError('bag player 서비스 또는 SLAM 구독자 준비 시간 초과')
            spin()
            ready = (resume.service_is_ready() and node.count_subscribers('/scan') >= 1
                     and node.count_publishers('/clock') == 1
                     and node.count_publishers('/odom') == 1)
            if ready:
                ready_at = ready_at or time.monotonic()
                if time.monotonic() - ready_at >= 1.0:
                    break
            else:
                ready_at = None
        parameters = start(['ros2', 'param', 'dump', options.slam_node], 'effective_params')
        wait_child(parameters, 15.0, 'effective_params')
        resumed = resume.call_async(Resume.Request())
        deadline = time.monotonic() + 10.0
        while not resumed.done():
            if time.monotonic() > deadline:
                raise RuntimeError('bag resume 응답 시간 초과')
            spin()
        if resumed.exception() is not None:
            raise RuntimeError('bag resume 서비스 실패: ' + str(resumed.exception()))
        started = time.monotonic()
        deadline = started + filter_report['record_duration_sec'] / options.rate + 60.0
        while player.poll() is None:
            if slam.poll() is not None:
                raise RuntimeError('재생 중 SLAM이 종료되었습니다')
            if time.monotonic() > deadline:
                raise RuntimeError('bag 재생 벽시계 제한 시간 초과')
            spin()
            if node.count_publishers('/clock') > 1:
                raise RuntimeError('재생 중 /clock 발행자가 중복되었습니다')
        if player.returncode != 0:
            raise RuntimeError(f'bag player 종료 코드 {player.returncode}')
        # EOF에서는 원본 clock 정지가 정상이다. 마지막 TF 콜백을 벽시계로 drain.
        drain_until = time.monotonic() + 2.5
        while time.monotonic() < drain_until:
            spin()
        observation = node.finish(output, filter_report['kept_counts']['/odom'],
                                  filter_report['last_odom_header_stamp_ns'])
        observer_finished = True
        status['observation'] = observation
        status['elapsed_playback_wall_sec'] = time.monotonic() - started
        status['trajectory_valid'] = observation['trajectory_valid']
        status['invalid_reasons'] = (observation['trajectory_invalid_reasons']
                                     + observation['map_issues'])
        status['last_clock_ns'] = observation['last_clock_ns']
        status['last_map_stamp_ns'] = observation['last_map_stamp_ns']
        if observation['initial_map_to_world'] is None or node.map is None:
            raise RuntimeError('초기 고정 정합 또는 지도를 확보하지 못했습니다')
        saver = start(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                       '-f', str(output / 'map'), '--ros-args',
                       '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=10.0'], 'save_map')
        wait_child(saver, 30.0, 'save_map')
        metrics = evaluate_map(output / 'map.yaml', inputs / 'truth.yaml',
                               observation['initial_map_to_world'])
        (output / 'metrics.json').write_text(
            json.dumps(metrics, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        trajectory = analyze_trajectory(output / 'trajectory.csv')
        (output / 'trajectory_metrics.json').write_text(
            json.dumps(trajectory, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        status['map_metrics'] = metrics
        status['trajectory_metrics'] = trajectory
        if (output / 'graph.jsonl').is_file():
            status['graph_motion'] = analyze_graph(output / 'graph.jsonl')
            (output / 'graph_motion.json').write_text(
                json.dumps(status['graph_motion'], ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        if status['invalid_reasons']:
            raise RuntimeError('replay 유효성 검사 실패: ' + '; '.join(status['invalid_reasons']))
        status['status'] = 'completed'
        status['trajectory_valid'] = observation['trajectory_valid']
        print(f'재생 평가 완료: {output}', flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        status['status'] = 'failed'
        status['error'] = str(exc) or type(exc).__name__
        print('재생 평가 실패: ' + status['error'], file=sys.stderr, flush=True)
        return 1
    finally:
        status['cleanup_errors'] = cleanup_children(children, handles)
        if status['cleanup_errors']:
            status['status'] = 'failed'
            status['error'] = '자식 프로세스 정리 실패: ' + '; '.join(status['cleanup_errors'])
        if node is not None:
            if not observer_finished and filter_report is not None:
                try:
                    status['observation'] = node.finish(
                        output, filter_report['kept_counts'].get('/odom', 0),
                        filter_report.get('last_odom_header_stamp_ns'))
                except Exception as exc:
                    status['observation_error'] = str(exc)
            node.destroy_node()
        if ros_initialized and rclpy.ok():
            rclpy.shutdown()
        save_manifest()
        if status['cleanup_errors']:
            return 1


if __name__ == '__main__':
    sys.exit(main())
