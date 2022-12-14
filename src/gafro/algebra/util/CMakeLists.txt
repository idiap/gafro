target_sources(${PROJECT_NAME}
    PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR}/util.cpp
)

install(
    FILES 
        Bitset.hpp
    DESTINATION 
        ${GAFRO_PREFIX_INCLUDE}/${PROJECT_NAME}/algebra/util
    COMPONENT 
        Devel
)