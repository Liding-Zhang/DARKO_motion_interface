
dir=$(find /home/ -name RbkairosFranka -type d)
source /opt/ros/noetic/setup.bash
source $dir/devel/setup.bash

alacritty -e roslaunch rbkairos_sim_bringup rbkairos_franka.launch moveit_movegroup_a:=true &

if [[ $# != 0 && "$1" == "graph" ]]; then
	rqt_graph 2> /dev/null &
fi
