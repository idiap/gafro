/*
    Copyright (c) 2022 Idiap Research Institute, http://www.idiap.ch/
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of gafro.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

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