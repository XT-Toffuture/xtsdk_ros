#!/bin/bash
echo "-----sel ros -$1-----"

if [ "$1" = "1" ]; then
	cp -f selros/package_ros1.xml package.xml
	cp -f selros/CMakeLists_ros1.txt CMakeLists.txt
elif [ "$1" = "2" -o "$1" = "" ]; then	
	cp -f selros/package_ros2.xml package.xml
	cp -f selros/CMakeLists_ros2.txt CMakeLists.txt
fi
