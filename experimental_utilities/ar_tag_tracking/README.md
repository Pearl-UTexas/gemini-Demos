# Extra files to conduct experiments on Gemini robot using ar_track_alvar

## Steps to use ar_track_alvar for the project
Install ar_track_alvar package
Bundle file: copy bundles/test_2.xml to the bundles folder of ar_track_alvar package
Launch file: copy launch/gemini_bundle.launch file to the launch folder of the package and then launch gemini_bundle.launch

### If working with Kinect v1.
1. Install openKinect from https://openkinect.org/wiki/Getting_Started
2. Install kinect v1 ros drivers. Currently being used, freenect_stack: http://wiki.ros.org/freenect_stack
3. launch kinect driver as: roslaunch freenect_launch freenect.launch
4. Change the image and camera info topic in the ar_track_alvar launch files: 
	<arg name="cam_image_topic" default="/camera/depth_registered/points" />
	<arg name="cam_info_topic" default="/camera/rgb/camera_info" />	
5. To view the tags in rviz:
* Change the base frame to /camera_link
* Change the corresponding topics

