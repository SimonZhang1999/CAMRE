gnome-terminal -x bash -c "source ~/CAMRE/devel/setup.bash; roslaunch cure_planner three_mir.launch"
sleep 3
gnome-terminal -x bash -c "source ~/CAMRE/devel/setup.bash; roslaunch cure_planner d_exploration.launch "
sleep 2
gnome-terminal -x bash -c "source ~/CAMRE/devel/setup.bash; roslaunch cure_planner frontier_assigner.launch "