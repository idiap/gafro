target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/expressions.cpp
)

# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            AbstractExpression.hpp
            AbstractExpression.hxx

            Expression.hpp
            UnaryExpression.hpp
            BinaryExpression.hpp
            Assign.hpp
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
            CommutatorProduct.hpp
            RotorExponential.hpp
            MotorLogarithm.hpp
            MotorExponential.hpp
            Reflection.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/${PROJECT_NAME}/algebra/expressions
        COMPONENT
            Devel
    )
endif()