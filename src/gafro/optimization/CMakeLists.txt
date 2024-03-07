target_sources(${PROJECT_NAME}
    PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR}/optimization.cpp
)

# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES 
            SingleManipulatorTarget.hpp
            SingleManipulatorDualTarget.hpp
            SingleManipulatorMotorCost.hpp
        DESTINATION 
            ${GAFRO_PREFIX_INCLUDE}/${PROJECT_NAME}/optimization
        COMPONENT
            Devel
    )
endif()
