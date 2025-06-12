// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/GeometricProduct.hpp>
#include <gafro/algebra/UnaryExpression.hpp>

namespace gafro
{

    template <class M>
    class DualPoincare : public UnaryExpression<DualPoincare<M>, M, typename M::PoincareDual>
    {
      public:
        using Base = UnaryExpression<DualPoincare<M>, M, typename M::PoincareDual>;

        using Vtype = typename M::Vtype;

        using Type = typename M::PoincareDual;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;
        constexpr static auto has = Type::has;

        DualPoincare(const M &m1) : Base(m1) {}

        DualPoincare(M &&m1) : Base(std::move(m1)) {}

        virtual ~DualPoincare() = default;

        template <int blade>
            requires(has(blade))  //
        Vtype get() const
        {
            return this->operand().template get<M::MAlgebra::BladeBitmap::template getPoincareDual<blade>()>();
        }

      protected:
      private:
    };

}  // namespace gafro