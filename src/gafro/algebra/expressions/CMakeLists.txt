target_sources(${PROJECT_NAME}
    PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR}/expressions.cpp
)

install(
    FILES 
        Cast.hpp

        Sum.hpp
        
        Product.hpp
        
        GeometricProduct.hpp
        GeometricProductCayleyTable.hpp
        
        InnerProduct.hpp
        InnerProductCayleyTable.hpp

        OuterProduct.hpp
        OuterProductCayleyTable.hpp

        Reverse.hpp
        Inverse.hpp
        Dual.hpp

        SandwichProduct.hpp
        Expression.hpp
        MotorLogarithm.hpp
        MotorExponential.hpp
        Reflection.hpp
    DESTINATION 
        ${GAFRO_PREFIX_INCLUDE}/${PROJECT_NAME}/algebra/expressions
    COMPONENT 
        Devel
)