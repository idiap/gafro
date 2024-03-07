# Geometric Algebra For RObotics

This library provides a geometric algebra tools targeted towards robotics applications. It includes various computations for the kinematics and dynamics of serial manipulators as well as optimal control.  

Please visit https://gitlab.com/gafro in order to find the entire *gafro* software stack.

## Installation

	mkdir build && cd build
	cmake ..
	make
	sudo make install

## Usage

gafro can be used either standalone or within a catkin workspace. In both cases it can be used in a CMakeLists.txt as follows:

	find_package(gafro REQUIRED)
	
	target_link_libraries(target gafro::gafro) 

## Background

You can find the accompanying article [here](http://arxiv.org/abs/2212.07237) and more information on our [website](https://geometric-algebra.tobiloew.ch/). 

## How to cite

If you use *gafro* in your research, please cite the

	@article{loewGeometricAlgebraOptimal2023,
	  title = {Geometric {{Algebra}} for {{Optimal Control}} with {{Applications}} in {{Manipulation Tasks}}},
	  author = {L\"ow, Tobias and Calinon, Sylvain},
	  date = {2023},
	  journal = {IEEE Transactions on Robotics},
	  doi = {10.1109/TRO.2023.3277282}
	}