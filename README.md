to run this 

ros2 launch ball_detector ball_detector.launch.py 
ros2 launch golf_robot_control ball_follow_launch.py 
ros2 launch wheel_serial_bridge wheel_serial_bridge_launch.py 
ros2 run my_teleop_robot teleop_node  
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run rosboard rosboard_node 
ros2 launch ydlidar_ros2_driver Tmini_launch.py 
ros2 launch pole_tracker pole_tracker.launch.py 


反省
- カメラが近くが見えない
- 近く用のセンサ
- 補助輪が弱い
- クラブが弱い
- 緑に弱い（何故か）
- 学習データがない
- 