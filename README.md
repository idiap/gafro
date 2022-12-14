# Geometric Algebra For RObotics

This library provides a geometric algebra tools targeted towards robotics applications. However it proves a proper build target and can be easily included in a catkin workspace to use it with ROS. 

## Usage
gafro can be used either standalone or within a catkin workspace. In both cases it can used in a CMakeLists.txt as follows:

	find_package(gafro REQUIRED)

	target_link_libraries(target gafro::gafro) 

