# nav_novamob
Novamob navigation package with gesture recognition

Test in simulation:

1st Terminal Window 4 terminals: 

host_machine_ws: 

	ros2 launch linorobot2_description description.launch.py use_sim_time:=true
 
	ros2 launch linorobot2_gazebo gazebo.launch.py sim:=true
 
	ros2 launch linorobot2_navigation navigation.launch.py map:=./o_meu_mapa.yaml rviz:=true sim:=true
 
	ros2 run nav_novamob closed_hand_node
	
2nd Terminal Window 2 terminals:

tese/dev_ws: 

	ros2 run gesture_recognition webcam_publisher  (realsense node substitutes this)
 
	ros2 run gesture_recognition hand_gesture_node (has absolute paths)
