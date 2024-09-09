target_sources(${PROJECT_NAME}
    PRIVATE
)

add_subdirectory(cga)

# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            Algebra.hpp
            Grade.hpp

            AbstractMultivector.hpp
            AbstractMultivector.hxx
            Multivector.hpp
            Multivector.hxx

            Operators.hxx

            MultivectorMatrix.hpp

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
            Reflection.hpp

            TypeTraits.hpp
            Bitset.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/algebra
        COMPONENT
            Devel
    )
endif()