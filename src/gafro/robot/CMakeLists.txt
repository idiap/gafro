target_sources(${PROJECT_NAME}
	PRIVATE 
		${CMAKE_CURRENT_LIST_DIR}/robot.cpp
		${CMAKE_CURRENT_LIST_DIR}/Joint.hpp
		${CMAKE_CURRENT_LIST_DIR}/PrismaticJoint.hpp
		${CMAKE_CURRENT_LIST_DIR}/Link.hpp
		${CMAKE_CURRENT_LIST_DIR}/Manipulator.hpp
)

install(
	FILES 
		Joint.hpp
		Joint.hxx

		PrismaticJoint.hpp
		PrismaticJoint.hxx

		Link.hpp
		Link.hxx
		
		Manipulator.hpp
		Manipulator.hxx

		EndEffector.hpp

		FrankaEmikaRobot.hpp
		FrankaEmikaHand.hpp
		
		# UniversalRobot10.hpp
	DESTINATION 
		${GAFRO_PREFIX_INCLUDE}/gafro/robot
	COMPONENT
		Devel
)