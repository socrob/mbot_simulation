# Cyton_manipulator

[How to get the Cyton Gamma 300 working in ROS](http://www.bandilabs.com/2014/11/13/get-cyton-gamma-300-working-ros/)

[cyton_gamma_1500_description (Braço no Rviz)](http://wiki.ros.org/cyton_gamma_1500_description)


`roslaunch pick_place_demo Manipulator.launch`

`roslaunch cyton_gama_1500_moveit_conf moveit_reviz.launch`

# Mbot Simulator

To configure gazebo, add the following code to ~/.bashrc, while using a text editor.

`export LC_NUMERIC=C`

Compile the workspace using:

`catkin_make`

To run the simulator, type this code into the terminal:

`roslaunch mbot_simulator mbot_sim.launch`

_Important remark: [URDF vs SDF](http://answers.gazebosim.org/question/62/sdf-vs-urdf-what-should-one-use/)_
