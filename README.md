in rosws
```
colcon build
source install/setup.bash
```
running code (edit yaml first with correct data)
```
ros2 launch camera_test_gt gt_test.launch.py
```
test with marker pose memorisation
```
ros2 launch camera_test_gt gt_test_mem.launch.py
```

2 ways to get distortion
1. Calibrate using my code
```
python3 other_util_code/camera_calibratr.py --rows 9 --cols 7 --size 18.5 --id 2 --num 40 --interval 60
```

2. Calibration of camera using pre made software (might be more precise)
Put correct ros distro!
```
sudo apt install ros-<jazzy>-v4l2-camera
```
sudo apt install ros-<jazzy>-camera-calibration
```

set correct id!
```
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args \
  -p video_device:="/dev/video2" \
  -p focus_auto:=false \
  -p focus_absolute:=0
```
set bord parametars
```
ros2 run camera_calibration cameracalibrator   --size 7x9   --square 0.0185  --ros-args -r image:=/image_raw
```
Resaults will be saved in '/tmp/calibrationdata.tar.gz'
