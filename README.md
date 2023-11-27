CSCE 452 Project 4: 
=====================

This package runs a small simulator for a differential drive robot.
This utilizes several ROS features, including `tf`, robot descriptions 
via `URDF`, and the `robot_state_publisher`.

Running the program
----------------
Run the following in different ubuntu windows, after running `colcon build` and `source install/setup.bash` from the workspace directory:
* `ros2 run project4 simulate_robot`
* `ros2 launch project4 demo.launch.py`
* `ros2 run rviz2 rviz2` - Be sure to include the following configurations in your `rviz2` window: Set your Fixed Frame to `odom`, add a `TF` visualization, and a `RobotModel` visualization with `/robot_description` set as the Description Topic
* `.../project4/project4a-bags/input#/input#_0.db3` - Runs the rosbag file to test