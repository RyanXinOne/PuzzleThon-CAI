export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=10.42.0.$1

if [ -z $1 ]; then
	echo No IP value
fi
