This is the readme for the perception handle of FCW package
This node actually implements road segmentation and vehicle tracking
Road segmentation is used to find out the drivable area ahead of the ego vehicle
Vehicle tracking is used to check if the ego vehicle is in a collison path towards the vehicle

The computed data is then published insde the fcw_perception message, which has the Most Important Object details as well as the maximum drivable distance in front of the ego vehicle.

To launch this node, build and source the workspace and then run,
roslaunch perception_fcw node.launch