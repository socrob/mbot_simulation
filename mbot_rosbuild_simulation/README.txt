
Follow these steps to run this simulator:
1. install these packages: ros-hydro-convex-decomposition ros-hydro-ivcon ros-hydro-controller-manager ros-hydro-transmission-interface ros-hydro-turtlebot-description
2. download and compile katana drivers, e.g.
   - source the scripts/katana_driver.url file
   - apply patches in scripts/katana_driver-v2.patch
   - compile the katana_description
3. compile scout_msgs, cobot_description, and this package
4. run using the launch/omni_cobot_gazebo.launch

(instructions written by Rodrigo Ventura)
