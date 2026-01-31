

<h1>ROS2 CODE, Not finished!!</h1>
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
<h1>Calibrating camera</h1>
2 ways to get distortion
1. Calibrate using my code
```
python3 other_util_code/camera_calibratr.py --rows 9 --cols 7 --size 18.5 --id 2 --num 40 --interval 60
```

2. Calibration of camera using pre made ROS software (might be more precise, and easyer to use for good calibration)
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
ros2 run camera_calibration cameracalibrator \
  --size 7x9 \
  --square 0.0185 \
  --no-service-check \
  --ros-args -r image:=/image_raw

```
Resaults will be saved in '/tmp/calibrationdata.tar.gz'

<h1>Testing coed</h1>
This code works just edit parametars in test_camera_odometry.py and run.
That runs camera marker SLAM, you should set parametars of marker and camera befor runing.
At least 1 marker needs to be seen at the start.
<h3>Keys to use</h3>
"q" exits the program, "m" enables maping, "l" locks maping

