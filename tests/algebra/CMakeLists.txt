target_sources(unittests
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/Circle.cpp
        ${CMAKE_CURRENT_LIST_DIR}/DirectionVector.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Line.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Motor.cpp
        ${CMAKE_CURRENT_LIST_DIR}/MotorGenerator.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Multivector.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Plane.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Point.cpp
        ${CMAKE_CURRENT_LIST_DIR}/PointPair.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Rotor.cpp
        ${CMAKE_CURRENT_LIST_DIR}/RotorGenerator.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Sphere.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Translator.cpp
        ${CMAKE_CURRENT_LIST_DIR}/TranslatorGenerator.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Vector.cpp
)
