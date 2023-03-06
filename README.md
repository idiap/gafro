# Geometric Algebra For RObotics

This library provides a geometric algebra tools targeted towards robotics applications. It includes various computations for the kinematics and dynamics of serial manipulators as well as optimal control.  

## Usage

gafro can be used either standalone or within a catkin workspace. In both cases it can be used in a CMakeLists.txt as follows:

	find_package(gafro REQUIRED)

	target_link_libraries(target gafro::gafro) 

## Background

You can find the accompanying article [here](http://arxiv.org/abs/2212.07237) and more information on our [website](https://tloew.gitlab.io/geometric_algebra/). 

## How to cite

If you use *gafro* in your research, please cite the

	@misc{loew2022_geometric_algebra,
	  title = {Geometric Algebra for Optimal Control with Applications in Manipulation Tasks},
	  author = {L\"ow, Tobias and Calinon, Sylvain},
	  publisher = {arXiv},
	  year = {2022},
	  doi = {10.48550/ARXIV.2212.07237},
	  url = {https://arxiv.org/abs/2212.07237},
	}