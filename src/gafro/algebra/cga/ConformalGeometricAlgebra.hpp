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

    template <int... index>
    std::ostream &operator<<(std::ostream &ostream, const ConformalGeometricAlgebra::Multivector<double, index...> &mv)
    {
        static const std::array<std::string, 32> BladeNames = { "",    "e0",   "e1",   "e01",   "e2",   "e02",   "e12",   "e012",
                                                                "e3",  "e03",  "e13",  "e013",  "e23",  "e023",  "e123",  "e0123",
                                                                "ei",  "e0i",  "e1i",  "e01i",  "e2i",  "e02i",  "e12i",  "e012i",
                                                                "e3i", "e03i", "e13i", "e013i", "e23i", "e023i", "e123i", "e0123i" };

        if (sizeof...(index) == 0)
        {
            ostream << 0;

            return ostream;
        }

        bool first = true;

        for (unsigned int k = 0; k < mv.vector().rows(); ++k)
        {
            if (abs(mv.vector().coeff(k, 0)) < 1e-10)
            {
                continue;
            }

            if (!first)
            {
                ostream << (mv.vector().coeff(k, 0) >= 0 ? " + " : " - ");

                ostream << abs(mv.vector().coeff(k, 0));
            }
            else
            {
                ostream << mv.vector().coeff(k, 0);

                first = false;
            }

            if (ConformalGeometricAlgebra::Multivector<double, index...>::blades()[k] > 0)
            {
                ostream << "*" << BladeNames[ConformalGeometricAlgebra::Multivector<double, index...>::blades()[k]];
            }
        }

        if (first)
        {
            ostream << 0;
        }

        return ostream;
    }

}  // namespace gafro