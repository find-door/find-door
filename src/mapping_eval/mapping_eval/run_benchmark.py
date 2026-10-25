"""격리된 indoor_v1 시뮬레이션을 반복 실행하고 산출물을 보존한다."""

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


def main(args=None):
    parser = argparse.ArgumentParser(description='indoor_v1 시뮬 전용 반복 매핑 평가')
    parser.add_argument('--output', required=True, type=Path, help='새 출력 폴더 (기존 폴더 거부)')
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
    if os.environ.get('ROS_DOMAIN_ID') != '12' or os.environ.get('GZ_PARTITION') != 'codex':
        parser.error('ROS_DOMAIN_ID=12 GZ_PARTITION=codex 환경에서 실행하세요')
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('mapping_eval'))
    slam_share = Path(get_package_share_directory('mapping_slam'))
    gazebo_share = Path(get_package_share_directory('mapping_gazebo'))
    route = share / 'config/indoor_v1_route.yaml'
    truth = share / 'config/indoor_v1_truth.yaml'
    params = (options.params_file.expanduser().resolve() if options.params_file
              else slam_share / 'config/slam.yaml')
    if not params.is_file():
        parser.error('SLAM 파라미터 파일이 없습니다: ' + str(params))
    source_files = {route.name: route, truth.name: truth, 'slam.yaml': params,
                    'indoor_v1.sdf': gazebo_share / 'worlds/indoor_v1.sdf'}
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
                                'record': options.record, 'ROS_DOMAIN_ID': 12,
                                'GZ_PARTITION': 'codex'},
              'code_sha256': {path.name: digest(path) for path in Path(__file__).resolve().parent.glob('*.py')},
              'input_sha256': hashes, 'wf_gazebo_sha256': external,
              'provisional_gates': {'p95_m_max': 0.15, 'precision_min': 0.95,
                                    'coverage_min': 0.85, 'tolerance_m': 0.1},
              'alignment': 'initial map_to_world only; no final alignment or scale fit',
              'limitations': 'Gazebo pose odom drives SLAM and route control; trajectory residual is not independent ATE',
              'runs': []}
    env = os.environ.copy()
    (output / 'manifest.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))

    def run(command, folder, name, timeout, expected=0):
        with (folder / (name + '.log')).open('w') as stream:
            child = subprocess.Popen(command, env=env, stdout=stream,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = child.wait(timeout=timeout)
            finally:
                stop_process(child)
        if code != expected:
            raise RuntimeError(f'{name}: 종료 코드 {code}, 로그 {folder / (name + ".log")}')

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
                log = (folder / 'simulation.log').open('w')
                handles.append(log)
                active.append(subprocess.Popen(
                    ['ros2', 'launch', 'mapping_slam', 'simulation.launch.py',
                     'world:=' + str(inputs / 'indoor_v1.sdf'),
                     'params_file:=' + str(inputs / 'slam.yaml')],
                    env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
                run(['ros2', 'run', 'mapping_eval', 'check_interfaces', '--timeout', '60',
                     '--require-map'], folder, 'preflight', 70)
                run(['ros2', 'param', 'dump', '/slam_toolbox'], folder, 'effective_params', 20)
                if options.record:
                    log = (folder / 'record.log').open('w')
                    handles.append(log)
                    active.append(subprocess.Popen(
                        ['ros2', 'bag', 'record', '-o', str(folder / 'bag'),
                         '/scan', '/odom', '/tf', '/tf_static', '/clock', '/cmd_vel'],
                        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
                    deadline = time.monotonic() + 15.0
                    while 'Recording...' not in (folder / 'record.log').read_text():
                        if active[-1].poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError('rosbag recorder 시작 실패: record.log 확인')
                        time.sleep(0.1)
                run(['ros2', 'run', 'mapping_eval', 'drive_route', '--confirm-simulation',
                     '--route', str(route), '--output', str(folder),
                     '--timeout', str(options.route_timeout), '--speed', str(options.speed)],
                    folder, 'drive', options.route_timeout + 20)
                run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(folder / 'map'),
                     '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=10.0'],
                    folder, 'save_map', 30)
                if options.record and active[-1].poll() is not None:
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
                report['runs'].append({'run': index + 1, 'elapsed_wall_s': time.monotonic() - started,
                                       'route': drive, 'metrics': metrics,
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
