#pragma once

#include <gafro/algebra/Algebra.hpp>

namespace gafro
{

    struct ConformalGeometricAlgebraMetric
    {
        constexpr static int dim = 5;

        template <int i, int j>
        constexpr static double get()
        {
            if constexpr (((i > 0) && (i < 4)) && ((j > 0) && (j < 4)) && (i == j))
            {
                return 1.0;
            }

            if constexpr ((i == 0 && j == 4) || (i == 4 && j == 0))
            {
                return -1.0;
            }

            return 0.0;
        }
    };

    using ConformalGeometricAlgebra = Algebra<ConformalGeometricAlgebraMetric>;

    template <class T, int... index>
    using Multivector = typename ConformalGeometricAlgebra::template Multivector<T, index...>;

}  // namespace gafro