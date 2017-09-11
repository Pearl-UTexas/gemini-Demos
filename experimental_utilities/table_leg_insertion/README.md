# Instructions for running basic utilities for the table leg insertion task

### Preparation to be done for the experiments
- On vector2, launch ar_track_alvar: roslaunch ar_track_alvar gemini_bundle.launch
- Launch moveit: `roslaunch hlpr_moveit_config hlpr_wpi_jaco_simple_moveit.launch`
- Go to the intial pose: `rosrun hlpr_moveit_config gemini_arm_tuck left` or `rosrun hlpr_moveit_config gemini_arm_tuck right`
- Record the observation noise covariance matrix: `roslaunch gemini_experimental_utils gemini_obs_fn_launch.launch`
- Update the observation noise covariance matrix in the HBLQR Planner.

#### Notes
- Grav_comp mode: Record the start position of the robot. Can use grav_comp mode for the arm: `rosservice call /jaco_arm/grav_comp true`
..* If you want to record some points on the trajecotry using ar_tags_alvar : `python experimental_utilities/table_leg_insertion/startup_utils`

### HBLQR execution: Nodes to be launched
- On vector2, launch ar_track_alvar: roslaunch ar_track_alvar gemini_bundle.launch
- On vector1, launch the main file: roslaunch hybrid_blqr table_leg_insertion_demo.launch

