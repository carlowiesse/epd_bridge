## **HOW TO RUN**

### Camera

1.1) ``sudo apt install ros-foxy-realsense2-camera``

1.2) ``source /opt/ros/foxy/setup.bash``

1.3) ``ros2 launch realsense2_camera rs_launch.py camera_name:=label_camera align_depth.enable:=true clip_distance:=-2. rgb_camera.profile:=640,480,15 depth_module.profile:=640,480,15 serial_no:=_145522060024``

For testing if the camera node is running:

a) ``ros2 topic echo /label_camera/color/image_raw``

b) ``ros2 run image_tools showimage --ros-args --remap /image:=/label_camera/color/image_raw``

1.4) ``ros2 launch realsense2_camera rs_launch.py camera_name:=pack_camera align_depth.enable:=true clip_distance:=-2. rgb_camera.profile:=640,480,15 depth_module.profile:=640,480,15 serial_no:=_007522060547``

For testing if the camera node is running:

a) ``ros2 topic echo /pack_camera/color/image_raw``

b) ``ros2 run image_tools showimage --ros-args --remap /image:=/pack_camera/color/image_raw``

1.5) ``ros2 launch realsense2_camera rs_launch.py camera_name:=safety_camera align_depth.enable:=true clip_distance:=-2. rgb_camera.profile:=640,480,30 depth_module.profile:=640,480,30 serial_no:=_145522060930``

For testing if the camera node is running:

a) ``ros2 topic echo /pack_camera/color/image_raw``

b) ``ros2 run image_tools showimage --ros-args --remap /image:=/safety_camera/color/image_raw``

### Communication with EMD

2.1) Go to location of EMD-EPD Bridge (new) package within your workspace

2.2) ``source install/setup.bash``

2.3) ``ros2 run epd_bridge box_vision``

2.4) ``ros2 run epd_bridge label_vision``

2.5) ``ros2 run epd_bridge tray_vision``

### EPD

3.1) Go to location of EPD package within EPD workspace

3.2) ``bash easy_perception_deployment/run.bash``

3.3) Click on 'Deploy' icon on EPD GUI

3.4) Make changes if needed and then click 'Run' icon on Deploy GUI

3.5) Enter system password (to enable sudo commands) and enter 'y' when asked to rebuild (if any changes were made)

3.6) Repeat steps 3.1-3.5 in a new terminal for all new instances of EPD

### Notes

- When calling any EMD services, the debug image will be saved in the EMD-EPD Bridge (new) package.
