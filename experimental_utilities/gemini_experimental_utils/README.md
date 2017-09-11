# Instructions for running basic utilities for the table leg insertion task

### Preparation to be done for the experiments
- On vector2, launch ar_track_alvar: `roslaunch ar_track_alvar gemini_bundle.launch`
- Launch moveit: `roslaunch hlpr_wpi_jaco_moveit_config hlpr_wpi_jaco_simple_moveit.launch`

..* If working with left arm
- Tuck the right arm: `rosrun hlpr_wpi_jaco_moveit_config arm_tuck right`
- Go to the intial pose: `rosrun hlpr_wpi_jaco_moveit_config gemini_arm_tuck left`
- Grab the leg: `rosrun gemini_experimental_utils gripper_utils.py`
..* Close with a high force (around 150 should be enough)
 
- Record the observation noise covariance matrix: `roslaunch gemini_experimental_utils gemini_obs_fn_launch.launch`
- Update the observation noise covariance matrix in the HBLQR Planner using npz files under data folder.

#### Notes
- Grav_comp mode: Record the start position of the robot. Can use grav_comp mode for the arm: `rosservice call /jaco_arm/grav_comp true`
..* If you want to record some points on the trajecotry using ar_tags_alvar : `python experimental_utilities/table_leg_insertion/startup_utils`

### HBLQR execution: Nodes to be launched
- On vector2, launch ar_track_alvar: roslaunch ar_track_alvar gemini_bundle.launch
- On vector1, launch the main file: roslaunch hybrid_blqr table_leg_insertion_demo.launch

