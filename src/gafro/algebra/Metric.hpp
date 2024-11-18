#pragma once

#include <Eigen/Core>
#include <gafro/algebra/Algebra.hpp>

namespace gafro
{

    template <int p, int q, int r>
    struct Metric
    {
        constexpr static int dim = p + q + r;

        template <int i, int j>
        constexpr static double get()
        {
            if constexpr ((i == j) && (i >= 0))
            {
                if constexpr (i < p)
                {
                    return 1.0;
                }

                if constexpr ((i >= p) && (i < p + q))
                {
                    return -1.0;
                }
            }

            return 0.0;
        }

        using Algebra = gafro::Algebra<Metric>;
    };

}  // namespace gafro