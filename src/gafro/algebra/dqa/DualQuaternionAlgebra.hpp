#pragma once

#include <gafro/algebra/Algebra.hpp>
#include <gafro/algebra/Multivector.hxx>
#include <map>

namespace gafro::dqa
{

    namespace idx
    {
        constexpr short scalar = 0b000;
        constexpr short i = 0b010;
        constexpr short j = 0b100;
        constexpr short k = 0b110;
        constexpr short e = 0b111;
        constexpr short ei = 0b101;
        constexpr short ej = 0b011;
        constexpr short ek = 0b001;

        static std::map<short, std::string> dq_blade_names = {
            { scalar, "" },  //
            { ek, "ek" },    //
            { i, "i" },      //
            { ej, "ej" },    //
            { j, "j" },      //
            { ei, "ei" },    //
            { k, "k" },      //
            { e, "e" },      //

        };
    }  // namespace idx

    namespace blades
    {
        constexpr short scalar = 0b000;
        constexpr short e0 = 0b001;
        constexpr short e1 = 0b010;
        constexpr short e01 = 0b011;
        constexpr short e2 = 0b100;
        constexpr short e02 = 0b101;
        constexpr short e12 = 0b110;
        constexpr short e012 = 0b111;
    }  // namespace blades

    struct DualQuaternionAlgebraMetric
    {
        constexpr static int dim = 3;

        template <int i, int j>
        constexpr static double get()
        {
            if constexpr (i == j)
            {
                if (i > 0)
                {
                    return -1.0;
                }
                else
                {
                    return 0.0;
                }
            }

            return 0.0;
        }
    };

    using DualQuaternionAlgebra = Algebra<DualQuaternionAlgebraMetric>;

    template <class T, int... index>
    using Multivector = typename DualQuaternionAlgebra::template Multivector<T, index...>;

    template <class T, int... left_index, int... right_index>
    auto join(const Multivector<T, left_index...> &lhs, const Multivector<T, right_index...> &rhs)
    {
        return (lhs.dualPoincare() ^ rhs.dualPoincare()).evaluate().dualPoincare();
    }

}  // namespace gafro::dqa

template <int... index>
std::ostream &operator<<(std::ostream &ostream, const gafro::dqa::Multivector<double, index...> &dq)
{
    bool first = true;

    for (unsigned int k = 0; k < dq.vector().rows(); ++k)
    {
        if (abs(dq.vector().coeff(k, 0)) < 1e-10)
        {
            continue;
        }

        if (!first)
        {
            ostream << (dq.vector().coeff(k, 0) >= 0 ? " + " : " - ");

            ostream << abs(dq.vector().coeff(k, 0));
        }
        else
        {
            ostream << dq.vector().coeff(k, 0);

            first = false;
        }

        if (gafro::dqa::Multivector<double, index...>::blades()[k] > 0)
        {
            ostream << "*" << gafro::dqa::idx::dq_blade_names[gafro::dqa::Multivector<double, index...>::blades()[k]];
        }
    }

    if (first)
    {
        ostream << 0;
    }

    return ostream;
}

template <class Derived, int... index>
std::ostream &operator<<(std::ostream &ostream, const gafro::Expression<Derived, gafro::dqa::Multivector<double, index...>> &expression)
{
    ostream << expression.evaluate();

    return ostream;
}