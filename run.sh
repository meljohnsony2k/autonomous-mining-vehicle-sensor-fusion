gnome-terminal -- bash -c "source devel/setup.bash; roslaunch playground_gazebo playground.launch;exec bash"

sleep 2

gnome-terminal -- bash -c "source devel/setup.bash; roslaunch fusion ekf_gps_imu.launch;exec bash"

sleep 2
gnome-terminal -- bash -c "source devel/setup.bash; rosrun fusion live_plot.py;exec bash"
sleep 2

#Keep last
gnome-terminal -- bash -c "source devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"
