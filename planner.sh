gnome-terminal -x bash -c "source ~/CAMRE/devel/setup.bash; roslaunch planner traj_opt.launch robot_name:=robot_1"
sleep 1
gnome-terminal -x bash -c "source ~/CAMRE/devel/setup.bash; roslaunch planner traj_opt.launch robot_name:=robot_2"
sleep 1
gnome-terminal -x bash -c "source ~/CAMRE/devel/setup.bash; roslaunch planner traj_opt.launch robot_name:=robot_3"
sleep 1
gnome-terminal -x bash -c "source ~/CAMRE/devel/setup.bash; roslaunch planner traj_opt.launch robot_name:=robot_4"
