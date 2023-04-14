#!/bin/bash

x=$(ifconfig | grep -oE '10\.42\.0\.[0-9]+' | sed 's/10\.42\.0\.//' | head -n1)

if [ -n "$x" ]; then
	export ROS_MASTER_URI=http://10.42.0.1:11311
	export ROS_IP=10.42.0.$x
fi
