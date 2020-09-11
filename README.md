# line_follower_pkg_ros
line_follower package for ROS (with a test arena)


This Package is meant for holding a developing "A Line Following Bot" using the camera input and PID control.

It also contains testing Arena.


USAGE:(use "catkin build" first):
    roslaunch line_follower arena_launch

In a next terminal tab:
    rosrun rqt_reconfigure rqt_reconfigure
    
In next terminal tab:
    rosrun line_follower Movement.py
    
Then refresh the rqt_reconfigure window.
There select Movement and configure the bot accordingly.
There is also, a node for controlling the bot manually "teleop.py".
A node for viewing the image processing involved "threshold.py".
A node for automatic line_follower using cv(trackbars) for dynamic reconfiuration "AutoMove.py".
A node for viewing the camera input "CamRcv.py".
