# Simulator documentation

 In order to execute this simulator run the following steps:

* Uncompress simulator.zip and open a terminal inside catkin_ws
* Execute the following lines of code
  ```
  catkin_make
  source devel/setup.bash
  roslaunch puzzlebot_world puzzlebot_simple_world.launch
  ```
* Open a new terminal and run 
  ```
  source devel/setup.bash
  ```
Now you can use this terminal to interact with the simulator. Keep in mind that this instruction needs to be executed whenever you open a new terminal. Alternatively, you can add it to your .bashrc file. 
* The warm-up challenge is to develop a line following algorithm, to do so you can retrieve images from the topic /camera/image_raw, read wheel velocities with /wr and /wl and send velocity commands with /cmd_vel