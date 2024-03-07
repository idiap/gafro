target_sources(${PROJECT_NAME}
    PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR}/util.cpp
)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES 
            Bitset.hpp
            TypeTraits.hpp
        DESTINATION 
            ${GAFRO_PREFIX_INCLUDE}/${PROJECT_NAME}/algebra/util
        COMPONENT 
            Devel
    )
endif()
