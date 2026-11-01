"""격리된 실내 world 시뮬레이션을 반복 실행하고 산출물을 보존한다."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('유한한 양수가 필요합니다')
    return number


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stop_process(child):
    """직접 생성한 프로세스 그룹만 종료한다."""
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            child.poll()
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            child.poll()  # 종료한 그룹 리더를 회수하되 자식도 별도로 기다린다.
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                return True
            time.sleep(0.1)
    child.poll()
    try:
        os.killpg(child.pid, 0)
    except ProcessLookupError:
        return True
    return False


CAPTURE_TOPICS = ('/scan', '/odom', '/tf', '/tf_static', '/clock',
                  '/slam_toolbox/graph_visualization', '/pose', '/mapping_capture/events')


def recorder_ready(log_text):
    """Recording 표시와 실제 토픽 구독 준비를 함께 확인한다."""
    return all("Subscribed to topic '" + topic + "'" in log_text for topic in CAPTURE_TOPICS)


def wait_until(predicate, children, timeout, label):
    deadline = time.monotonic() + timeout
    while True:
        if any(child.poll() is not None for child in children):
            raise RuntimeError(label + ': 프로세스 조기 종료')
        if predicate():
            return
        if time.monotonic() > deadline:
            raise RuntimeError(label + ': 제한 시간 초과')
        time.sleep(0.05)


def main(args=None):
    parser = argparse.ArgumentParser(description='실내 world 시뮬 전용 반복 매핑 평가')
    parser.add_argument('--output', required=True, type=Path, help='새 출력 폴더 (기존 폴더 거부)')
    parser.add_argument('--world', choices=('indoor_v1', 'indoor_v2'), default='indoor_v1')
    parser.add_argument('--params-file', type=Path, help='비교 실험용 SLAM YAML (입력 사본을 보존)')
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--speed', type=positive, default=0.35)
    parser.add_argument('--route-timeout', type=positive, default=240.0)
    parser.add_argument('--record', action='store_true', help='LiDAR·odom·TF·clock·명령 rosbag 기록')
    parser.add_argument('--confirm-simulation', action='store_true')
    options = parser.parse_args(args)
    if not options.confirm_simulation:
        parser.error('--confirm-simulation을 명시해야 합니다 (실물 실행 금지)')
    if not 1 <= options.runs <= 20 or options.speed > 0.5:
        parser.error('runs는 1~20, speed는 0보다 크고 0.5 이하이어야 합니다')
    from mapping_eval.drive_route import agent_environment
    try:
        isolation = agent_environment(os.environ)
    except ValueError as error:
        parser.error(str(error))
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('mapping_eval'))
    slam_share = Path(get_package_share_directory('mapping_slam'))
    gazebo_share = Path(get_package_share_directory('mapping_gazebo'))
    route = share / f'config/{options.world}_route.yaml'
    truth = share / f'config/{options.world}_truth.yaml'
    params = (options.params_file.expanduser().resolve() if options.params_file
              else slam_share / 'config/slam.yaml')
    if not params.is_file():
        parser.error('SLAM 파라미터 파일이 없습니다: ' + str(params))
    source_files = {route.name: route, truth.name: truth, 'slam.yaml': params,
                    options.world + '.sdf': gazebo_share / f'worlds/{options.world}.sdf'}
    output = options.output.expanduser().resolve()
    try:
        output.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        parser.error('기존 출력 폴더를 덮어쓰지 않습니다: ' + str(output))
    inputs = output / 'inputs'
    inputs.mkdir()
    hashes = {}
    for name, path in source_files.items():
        (inputs / name).write_bytes(path.read_bytes())
        hashes[name] = digest(inputs / name)
    route = inputs / route.name
    truth = inputs / truth.name
    wf_share = Path(get_package_share_directory('wf_gazebo'))
    external = {}
    for relative in ('launch/spawn_robot.launch.py', 'config/gz_bridge.yaml',
                     'urdf/rosorin.gz.xacro', 'urdf/mecanum.xacro'):
        path = wf_share / relative
        external[relative] = digest(path) if path.exists() else None
    report = {'status': 'running', 'runs_requested': options.runs,
              'configuration': {'speed_mps': options.speed,
                                'route_timeout_wall_s': options.route_timeout,
                                'record': options.record, **isolation},
              'code_sha256': {path.name: digest(path) for path in Path(__file__).resolve().parent.glob('*.py')},
              'input_sha256': hashes, 'wf_gazebo_sha256': external,
              'provisional_gates': {'p95_m_max': 0.15, 'precision_min': 0.95,
                                    'coverage_min': 0.85, 'tolerance_m': 0.1},
              'alignment': 'initial map_to_world only; no final alignment or scale fit',
              'limitations': 'Gazebo pose odom drives SLAM and route control; trajectory residual is not independent ATE',
              'runs': []}
    if options.record:
        qos_source = share / 'config/startup_capture_qos.yaml'
        (inputs / qos_source.name).write_bytes(qos_source.read_bytes())
        hashes[qos_source.name] = digest(inputs / qos_source.name)
    report['capture_policy'] = ('diagnostics, recorder subscriptions, Gazebo sensors, SLAM, route'
                                if options.record else 'integrated simulation without recording')
    env = os.environ.copy()
    (output / 'manifest.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))

    def run(command, folder, name, timeout, expected=0):
        with (folder / (name + '.log')).open('w') as stream:
            child = subprocess.Popen(command, env=env, stdout=stream,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = child.wait(timeout=timeout)
            finally:
                if stop_process(child) is False:
                    raise RuntimeError(name + ': 프로세스 그룹 정리 실패')
        if code != expected:
            raise RuntimeError(f'{name}: 종료 코드 {code}, 로그 {folder / (name + ".log")}')

    def start(command, folder, name):
        log = (folder / (name + '.log')).open('w')
        handles.append(log)
        child = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        active.append(child)
        return child

    def mark(folder, phase):
        run(['ros2', 'service', 'call', '/mapping_capture/mark_' + phase,
             'std_srvs/srv/Trigger', '{}'], folder, 'mark_' + phase, 20)
        events = [json.loads(line) for line in (folder / 'diagnostics/events.jsonl').read_text().splitlines()]
        if not events or events[-1]['event'] != phase:
            raise RuntimeError('진단 단계 이벤트 확인 실패: ' + phase)
        return events[-1]

    active = []
    handles = []
    try:
        # 기존 실행을 종료하거나 같이 제어하지 않는다. 같은 도메인이 비어 있어야 한다.
        import rclpy
        rclpy.init(args=[])
        probe = rclpy.create_node('mapping_benchmark_domain_probe')
        deadline = time.monotonic() + 3.0
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.1)
                if probe.count_publishers('/clock') or probe.count_publishers('/odom'):
                    raise RuntimeError('도메인 12에 기존 시뮬/로봇이 있습니다. 해당 작업 종료 후 실행하세요')
        finally:
            probe.destroy_node()
            rclpy.shutdown()
        for index in range(options.runs):
            folder = output / f'run_{index + 1:02d}'
            folder.mkdir()
            env['ROS_LOG_DIR'] = str(folder / 'roslogs')
            print(f'실험 {index + 1}/{options.runs}: {folder}', flush=True)
            started = time.monotonic()
            try:
                capture = None
                if options.record:
                    import yaml
                    configuration = yaml.safe_load((inputs / 'slam.yaml').read_text())
                    tf_timeout = float(configuration['slam_toolbox']['ros__parameters'].get('transform_timeout', 0.2))
                    diagnostics = start(
                        ['ros2', 'run', 'mapping_eval', 'capture_diagnostics', '--output',
                         str(folder / 'diagnostics'), '--transform-timeout', str(tf_timeout),
                         '--confirm-simulation'], folder, 'diagnostics')
                    wait_until(lambda: (folder / 'diagnostics/ready.json').is_file(),
                               [diagnostics], 20, '진단 수집기 준비')
                    recorder = start(
                        ['ros2', 'bag', 'record', '-o', str(folder / 'bag'),
                         '--include-unpublished-topics', '--polling-interval', '10',
                         '--qos-profile-overrides-path', str(inputs / 'startup_capture_qos.yaml'),
                         *CAPTURE_TOPICS, '/cmd_vel', '/rosout'], folder, 'record')
                    wait_until(lambda: recorder_ready((folder / 'record.log').read_text()),
                               [diagnostics, recorder], 20, 'SLAM 시작 전 bag 구독 준비')
                    simulation = start(
                        ['ros2', 'launch', 'mapping_gazebo', 'indoor.launch.py',
                         'gui:=false', 'headless_rendering:=true',
                         'world:=' + str(inputs / (options.world + '.sdf'))], folder, 'simulation')
                    run(['ros2', 'run', 'mapping_eval', 'check_interfaces', '--timeout', '60'],
                        folder, 'sensor_preflight', 70)
                    capture = {'slam_start': mark(folder, 'slam_start'),
                               'slam_process_requested_monotonic_ns': time.monotonic_ns()}
                    slam = start(
                        ['ros2', 'launch', 'mapping_slam', 'mapping.launch.py',
                         'use_sim_time:=true', 'params_file:=' + str(inputs / 'slam.yaml')],
                        folder, 'slam')
                else:
                    simulation = start(
                        ['ros2', 'launch', 'mapping_slam', 'simulation.launch.py',
                         'world:=' + str(inputs / (options.world + '.sdf')),
                         'params_file:=' + str(inputs / 'slam.yaml')], folder, 'simulation')
                run(['ros2', 'run', 'mapping_eval', 'check_interfaces', '--timeout', '60',
                     '--require-map'], folder, 'preflight', 70)
                run(['ros2', 'param', 'dump', '/slam_toolbox'], folder, 'effective_params', 20)
                if capture is not None:
                    capture['drive_start'] = mark(folder, 'drive_start')
                run(['ros2', 'run', 'mapping_eval', 'drive_route', '--confirm-simulation',
                     '--route', str(route), '--output', str(folder),
                     '--timeout', str(options.route_timeout), '--speed', str(options.speed)],
                    folder, 'drive', options.route_timeout + 20)
                run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(folder / 'map'),
                     '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=10.0'],
                    folder, 'save_map', 30)
                if options.record and recorder.poll() is not None:
                    raise RuntimeError('rosbag recorder가 주행 도중 종료됨')
                drive = json.loads((folder / 'route_result.json').read_text())
                transform = drive['initial_map_to_world']
                run(['ros2', 'run', 'mapping_eval', 'evaluate_map', '--map', str(folder / 'map.yaml'),
                     '--truth', str(truth), '--map-to-world', *[format(x, ".17f") for x in transform],
                     '--output', str(folder / 'metrics.json'), '--tolerance', '0.1'],
                    folder, 'metrics', 30)
                metrics = json.loads((folder / 'metrics.json').read_text())
                gates = {
                    'wall_p95': metrics['occupied_to_truth']['p95_m'] <= 0.15,
                    'wall_precision': metrics['occupied_to_truth']['precision_at_tolerance'] >= 0.95,
                    'wall_coverage': metrics['truth_to_occupied']['coverage_at_tolerance'] >= 0.85,
                }
                if capture is not None:
                    capture['capture_end'] = mark(folder, 'capture_end')
                    for child in (slam, simulation):
                        if stop_process(child) is False:
                            raise RuntimeError('센서/SLAM 종료 실패')
                        active.remove(child)
                    time.sleep(0.5)  # 센서 종료 후 DDS 콜백과 기록 큐를 처리한다.
                    for child in (recorder, diagnostics):
                        if stop_process(child) is False:
                            raise RuntimeError('기록/진단 종료 실패')
                        active.remove(child)
                    from mapping_eval.startup_audit import audit_startup
                    audit = audit_startup(folder / 'bag', folder / 'diagnostics')
                    (folder / 'startup_audit.json').write_text(json.dumps(audit, indent=2, ensure_ascii=False))
                    capture['audit'] = audit
                    from mapping_eval.graph_motion import analyze_graph
                    capture['graph_motion'] = analyze_graph(folder / 'diagnostics/graph.jsonl')
                    (folder / 'graph_motion.json').write_text(
                        json.dumps(capture['graph_motion'], indent=2, ensure_ascii=False))
                    if not audit['startup_coverage_valid'] or not audit['diagnostic_valid']:
                        raise RuntimeError('초기 입력 기록 coverage 검사 실패: startup_audit.json 확인')
                report['runs'].append({'run': index + 1, 'elapsed_wall_s': time.monotonic() - started,
                                       'route': drive, 'metrics': metrics, 'capture': capture,
                                       'quality_gate': gates, 'quality_pass': all(gates.values())})
            finally:
                failures = []
                for child in reversed(active):
                    try:
                        if stop_process(child) is False:
                            failures.append(str(child.pid))
                    except Exception as exc:
                        failures.append(str(exc))
                active.clear()
                for handle in handles:
                    handle.close()
                handles.clear()
                if failures:
                    raise RuntimeError('프로세스 정리 실패: ' + ', '.join(failures))
            (output / 'manifest.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
        report['status'] = 'completed'
        report['quality_pass'] = all(row['quality_pass'] for row in report['runs'])
        report['aggregate'] = {}
        for key, section, metric in (
            ('wall_p95_m', 'occupied_to_truth', 'p95_m'),
            ('wall_rmse_m', 'occupied_to_truth', 'rmse_m'),
            ('wall_precision', 'occupied_to_truth', 'precision_at_tolerance'),
            ('wall_coverage', 'truth_to_occupied', 'coverage_at_tolerance'),
        ):
            values = [row['metrics'][section][metric] for row in report['runs']]
            report['aggregate'][key] = {'mean': statistics.mean(values),
                                        'min': min(values), 'max': max(values),
                                        'sample_std': statistics.stdev(values) if len(values) > 1 else None}
        print(f'완료: {options.runs}회, {output / "manifest.json"}', flush=True)
        return 0
    except (RuntimeError, OSError, ValueError, KeyError, subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
        report['status'] = 'failed'
        report['error'] = str(exc) or type(exc).__name__
        print('실패: ' + report['error'], file=sys.stderr, flush=True)
        return 1
    finally:
        for child in reversed(active):
            stop_process(child)
        for handle in handles:
            handle.close()
        (output / 'manifest.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    sys.exit(main())
