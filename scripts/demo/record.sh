#!/bin/bash
# 사용: record.sh OUTDIR [drive]   drive 없으면 짧은 시험 녹화
OUT=$1; MODE=$2
S=$(cd "$(dirname "$0")" && pwd); WS=$(cd "$S/../.." && pwd)
source /opt/ros/humble/setup.bash; source ~/worker_following_ws/install/setup.bash; source $WS/install/setup.bash
export ROS_DOMAIN_ID=11 GZ_PARTITION=claude DISPLAY=${DISPLAY:-:1}
mkdir -p $OUT/cam $OUT/rviz; cd $WS
set -m
ros2 launch mapping_slam simulation.launch.py world:=$WS/install/mapping_gazebo/share/mapping_gazebo/worlds/indoor_demo.sdf > $OUT/sim.log 2>&1 & SIM=$!
ros2 run mapping_eval check_interfaces --timeout 60 --require-map > $OUT/check.log 2>&1; echo "check exit $?"
rviz2 -d $S/demo.rviz > $OUT/rviz.log 2>&1 & RVIZ=$!
for i in $(seq 30); do XID=$(xwininfo -root -tree | grep -F 'demo.rviz - RViz"' | awk '{print $1}' | head -1); [ -n "$XID" ] && break; sleep 1; done
echo "xid=$XID"; sleep 4
python3 $S/slam_path.py > $OUT/path.log 2>&1 & PATHN=$!
python3 $S/cam_rec.py $OUT/cam > $OUT/cam.log 2>&1 & CAM=$!
gst-launch-1.0 -e ximagesrc xid=$XID use-damage=false show-pointer=false ! video/x-raw,framerate=10/1 ! videoconvert ! pngenc compression-level=1 ! multifilesink location=$OUT/rviz/%05d.png > $OUT/gst.log 2>&1 & GST=$!
if [ "$MODE" = drive ]; then
  sleep 2
  ros2 run mapping_eval drive_route --route src/mapping_eval/config/indoor_v1_route.yaml --output $OUT/drive --confirm-simulation > $OUT/drive.log 2>&1; echo "drive exit $?"
  sleep 3
else
  sleep 4
fi
kill -INT -- -$GST; kill -INT -- -$CAM; sleep 2
kill -INT -- -$PATHN; kill -INT -- -$RVIZ; kill -INT -- -$SIM
sleep 5; for p in $GST $CAM $PATHN $RVIZ $SIM; do kill -0 -- -$p 2>/dev/null && { echo "TERM $p"; kill -TERM -- -$p; }; done
echo "cam $(ls $OUT/cam | wc -l) rviz $(ls $OUT/rviz | wc -l)"
