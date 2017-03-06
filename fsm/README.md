# Running the Hand-off demo in simulation
Instructions for running the Hand-off FSM in Gazebo simulation. Note that this simulation only tests navigation in the simulation room and not in a simulated basement as the actual demo.

##Prerequisites
1. Currently we support Ubuntu 14.04 with ROS Indigo
  - [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)

2. Install base vector packages [here](https://github.com/StanleyInnovation/vector_v1/wiki/Setup-Instructions).

3. Follow the simulation tutorial [here](https://github.com/HLP-R/hlpr_simulator/wiki/Getting%20Started).

4. Follow the perception tutorial [here](https://github.com/HLP-R/hlpr_perception/wiki).

5. Follow the manipulation tutorial [here](https://github.com/HLP-R/hlpr_manipulation/wiki/Getting%20Started).

6. Follow the navigation tutorial [here](https://github.com/StanleyInnovation/vector_v1/wiki/Navigation).

7. Follow the speech tutorial [here](https://github.com/HLP-R/hlpr_speech/wiki/Getting%20Started).

##Install missing packages
  - `sudo apt-get install python-espeak`
  - `sudo apt-get install ros-indigo-dynamixel-msgs`
  - `sudo apt-get install python-pip python-dev build-essential`
  - `sudo pip install aenum`

##Running the FSM
1. Close all windows or `source ~/.bashrc` in all windows.

2. Open six terminal windows.

3. Window 1: Launch Gazebo.
  - `roslaunch simulation vector_handoff_demo.launch`
  - If the purple cylinder is not on the table or is not there at all, delete the cylinder object from gazebo and respawn the cylinder with `roslaunch simulation spawn_cylinder.launch`

4. Window 2: Launch Rviz.
  - `roslaunch vector_viz view_robot.launch function:=map_nav`
  
5. Window 3: Launch the navigation service.
  - `roslaunch vector_navigation_apps 2d_map_nav_demo.launch map_file:=simulation_room sim:=true`
  - Provide a 2D pose estimate using the RVIZ window you just launched. The robot will localize by driving in a circle.

6. Window 4: Launch the perception service.
  - `roslaunch handoff_segmentation handoff_demo.launch`

7. Window 5: Launch the speech recognition service.
  - Connect a microphone or use the Kinect built-in microphone.
  - `rosrun hlpr_speech_recognition speech_recognizer`
  - OR Launch the Speech GUI interface
  - `rosrun hlpr_speech_recognition speech_gui`

8. Window 6: Launch the FSM script.
  - `cd` to the simlab repository
  - `cd handoff_demo/fsm/reorg_scripts`
  - `python main.py`

9. And you're done!

# Running the Handoff Demo on the Robot

Note: This has not yet been tried out! Use at your own risk!

## Disclaimer

Do not run the demo on the robot unless you verify the following. The steps to run assumes:

1. That you already have everything on your current machine 

2. The right bash files are sourced 

3. vector1 IP is correctly set

4. Everything is in order with the robot 

5. A map of the environment has been made 

6. Correct locations are set for navigation

7. You know the robot password

##Steps

1. Connect to the robot wifi: SIVectorS2

2. Make sure that your network configuration is correct before bash files are sourced
  - `ROBOT_NETWORK=wlan0`
  - `ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')`
  - `ROS_MASTER_URI=http://vector1:11311/`

3. Open up 8 terminals

4. Terminal 1: Connect to vector1 and launch the navigation
  - `ssh vector@vector1`
  - `vchk` to make sure everything is fine
  - Launch the navigation service: `roslaunch navigation gdc_basement_nav_demo.launch map_file:=gdc_basement`

5. Terminal 2: Run the teleoperation node
  - `roslaunch vector_remote_teleoperation vector_remote_teleoperation.launch`

6. Terminal 3: Launch wpi_jaco_wrapper
  - `ssh vector@vector1`
  - `roslaunch wpi_jaco_wrapper arm.launch`
  - Wait for the arm to be initialized

7. Terminal 4: Launch moveit
  - `ssh vector@vector1`
  - `roslaunch hlpr_wpi_jaco_moveit_config hlpr_wpi_jaco_simple_moveit.launch`
  - Wait to see the happy message

8. Terminal 5: Launch Rviz
  - `rosrun manipulation arm_upper_tuck` to put the arm in the upper tuck pose OR use the joystick to put the arm in the lower tuck pose if the fsm will not execute arm trajectories. It is easier to put the arm in lower tuck than upper tuck with the joystick
  - `roslaunch vector_viz view_robot.launch function:=map_nav`
  - Using the joystick, make sure that the robot will not hit anything when it rotates in place. 
  - Provide a 2D pose estimate using the RVIZ window you just launched. 
  - Make the robot localization makes sense. If it is not good, drive it around until it gets better.
  - If it is still not good, give it another pose. This time it will not rotate in place. Drive again if needed.
  - Now un-tick everything on RViz and add motion planning
  - When the OMPL is loaded, add the bigbox object to the scene, and have it align with the table
  - Make sure that you publish the scene

9. Terminal 6: Launch perception
  - `roslaunch handoff_segmentation handoff_demo.launch`

10. Terminal 7: Run the speech_recognizer 
  - `ssh vector@vector2` OR on the machine that the mic is connected to
  - `rosrun hlpr_speech_recognition speech_recognizer` OR
  - `rosrun hlpr_speech_recognition speech_gui` if you want to press the buttons on your local machine instead

11. Terminal 8: Run the fsm
  - `roscd fsm/reorg_scripts`
  - `python main.py`
  - In the future, this will be a `rosrun`

12. Tell the robot "Let's begin the experiment" and be ready to press the E-STOP

13. When/If robot return with the object, tell it "Open your hand" for the hand-off

14. Rejoice
