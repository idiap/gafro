target_sources(${PROJECT_NAME}
    PRIVATE
)

add_subdirectory(cga)
add_subdirectory(dqa)
add_subdirectory(pga)

# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            Algebra.hpp

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
            Conjugate.hpp
            SharpConjugate.hpp
            Inverse.hpp
            Dual.hpp
            DualPoincare.hpp

            SandwichProduct.hpp
            CommutatorProduct.hpp
            Reflection.hpp

            TypeTraits.hpp
            Bitset.hpp

            dqa.hpp
            pga.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/algebra
        COMPONENT
            Devel
    )
endif()