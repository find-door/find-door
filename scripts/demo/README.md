# 데모 영상 녹화

README의 [데모 GIF](../../docs/media/mapping_demo.gif)를 만드는 스크립트입니다. 소개 자료용이며 평가 도구가 아닙니다.

- world: [`indoor_demo`](../../src/mapping_gazebo/worlds/indoor_demo.sdf). 벽 충돌체는 `indoor_v1`과 같고 텍스처·가구를 더했습니다. 가구가 LiDAR에 보이므로 `indoor_v1` 정답 데이터로 평가하지 않습니다.
- 왼쪽: `/camera/color/image_raw` 원본. 오른쪽: RViz2 창 캡처(`slam_toolbox` 지도, LiDAR 스캔, 로봇 모델, TF `map → base_footprint`를 쌓은 추정 경로).

```bash
python3 scripts/demo/generate_demo_world.py   # world·텍스처 재생성 후 colcon build
scripts/demo/record.sh <출력 폴더> drive       # ROS_DOMAIN_ID=11, GZ_PARTITION=claude로 실행
python3 scripts/demo/compose.py <출력 폴더> <프레임 폴더> 3   # 3배속 좌우 합성
```

녹화 중 RViz2 창이 현재 화면(`DISPLAY`)에 뜹니다. GIF·MP4 인코딩에는 Pillow와 GStreamer를 사용했습니다.
