#pragma once

#include <gafro/algebra/AbstractExpression.hxx>
#include <gafro/algebra/Algebra.hpp>

namespace gafro::pga
{

    struct ProjectiveGeometricAlgebraMetric
    {
        constexpr static int dim = 4;

        template <int i, int j>
        constexpr static double get()
        {
            if constexpr (i == j)
            {
                if constexpr (i > 0)
                {
                    return 1.0;
                }
                else if (i == 0)
                {
                    return 0.0;
                }
            }

            return 0.0;
        }
    };

    using ProjectiveGeometricAlgebra = Algebra<ProjectiveGeometricAlgebraMetric>;

    template <class T, int... index>
    using Multivector = typename ProjectiveGeometricAlgebra::template Multivector<T, index...>;

    template <class T, int... left_index, int... right_index>
    auto meet(const Multivector<T, left_index...> &lhs, const Multivector<T, right_index...> &rhs)
    {
        return lhs ^ rhs;
    }

    template <class T, int... left_index, int... right_index>
    auto join(const Multivector<T, left_index...> &lhs, const Multivector<T, right_index...> &rhs)
    {
        return (lhs.dualPoincare() ^ rhs.dualPoincare()).evaluate().dualPoincare();
    }

    namespace blades
    {

        constexpr short scalar = 0b0000;
        constexpr short e0 = 0b0001;
        constexpr short e1 = 0b0010;
        constexpr short e01 = 0b0011;
        constexpr short e2 = 0b0100;
        constexpr short e02 = 0b0101;
        constexpr short e12 = 0b0110;
        constexpr short e012 = 0b0111;
        constexpr short e3 = 0b1000;
        constexpr short e03 = 0b1001;
        constexpr short e13 = 0b1010;
        constexpr short e013 = 0b1011;
        constexpr short e23 = 0b1100;
        constexpr short e023 = 0b1101;
        constexpr short e123 = 0b1110;
        constexpr short e0123 = 0b1111;
    }  // namespace blades

}  // namespace gafro::pga

template <int... index>
std::ostream &operator<<(std::ostream &ostream, const gafro::pga::Multivector<double, index...> &dq)
{
    static std::array<std::string, 16> dq_blade_names = { "",   "e0",  "e1",  "e01",  "e2",  "e02",  "e12",  "e012",
                                                          "e3", "e03", "e13", "e013", "e23", "e023", "e123", "e0123" };

    bool first = true;

    for (unsigned int k = 0; k < dq.vector().rows(); ++k)
    {
        if (fabs(dq.vector().coeff(k, 0)) < 1e-10)
        {
            continue;
        }

        if (!first)
        {
            ostream << (dq.vector().coeff(k, 0) >= 0 ? " + " : " - ");

            ostream << fabs(dq.vector().coeff(k, 0));
        }
        else
        {
            ostream << dq.vector().coeff(k, 0);

            first = false;
        }

        if (gafro::pga::Multivector<double, index...>::blades()[k] > 0)
        {
            ostream << "*" << dq_blade_names[gafro::pga::Multivector<double, index...>::blades()[k]];
        }
    }

    if (first)
    {
        ostream << 0;
    }

    return ostream;
}

template <class Derived, int... index>
std::ostream &operator<<(std::ostream &ostream, const gafro::Expression<Derived, gafro::pga::Multivector<double, index...>> &expression)
{
    ostream << expression.evaluate();

    return ostream;
}