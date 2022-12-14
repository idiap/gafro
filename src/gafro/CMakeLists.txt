configure_file(${CMAKE_CURRENT_LIST_DIR}/gafro_package_config.h.in "${CMAKE_CURRENT_BINARY_DIR}/gafro/gafro_package_config.hpp")

add_library(${PROJECT_NAME})

target_compile_options(${PROJECT_NAME}
	PUBLIC
    "-march=native"
    "-msse2"
    "-O3"
)


add_subdirectory(algebra)
add_subdirectory(physics)
# add_subdirectory(motion)
add_subdirectory(robot)
add_subdirectory(optimal_control)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${${PROJECT_NAME}_VERSION}
  SOVERSION 1
  INTERFACE_GAFRO_MAJOR_VERSION 1
)

find_package(Eigen3 REQUIRED)

message(${CMAKE_CURRENT_BINARY_DIR})

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<INSTALL_INTERFACE:${${PROJECT_NAME}_PREFIX_INCLUDE}>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}>
		$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>

		$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>/..
)

target_include_directories(${PROJECT_NAME} 
	SYSTEM PRIVATE
)

target_link_libraries(${PROJECT_NAME}
	PRIVATE 
		$<BUILD_INTERFACE:${EIGEN3_LIBRARIES}>
	PUBLIC
)

install(TARGETS ${PROJECT_NAME} EXPORT ${CMAKE_PROJECT_NAME}-config-targets
	LIBRARY DESTINATION lib/${PROJECT_NAME}/
	ARCHIVE DESTINATION lib/${PROJECT_NAME}/
	RUNTIME DESTINATION lib/${PROJECT_NAME}/
	INCLUDES DESTINATION ${GAFRO_PREFIX_INCLUDE}/
)

install(
	FILES 
		"${CMAKE_CURRENT_BINARY_DIR}/gafro/gafro_package_config.hpp" 
		gafro.hpp
		algebra.hpp
		motion.hpp
		robot.hpp
		physics.hpp
		optimal_control.hpp
	DESTINATION 
		${GAFRO_PREFIX_INCLUDE}/gafro
	COMPONENT 
		Devel
)