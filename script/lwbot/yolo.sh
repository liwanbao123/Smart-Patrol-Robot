source ../install/setup.bash
ros2 run yolov5_ros2 yolo_detect_2d --ros-args -p device:=cpu -p image_topic:=/lwbot_camera_raw -p pub_result_img:=True
