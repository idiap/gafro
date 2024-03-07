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

#include <gafro/algebra/Blades.hpp>
#include <gafro/algebra/expressions/BinaryExpression.hpp>
#include <gafro/algebra/expressions/GeometricProduct.hpp>

namespace gafro
{

    template <class M1, class M2>
    class CommutatorProduct : public Expression<CommutatorProduct<M1, M2>, M2>
    {
      public:
        using Type = M2;
        using Vtype = typename Type::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;

        // constexpr static auto map = Type::map;

        CommutatorProduct(const M1 &m1, const M2 &m2) : result_(m1 * m2) {}

        virtual ~CommutatorProduct() = default;

        template <int blade>
            requires(Type::has(blade))  //
        Vtype get() const
        {
            return result_.template get<blade>();
        }

      protected:
      private:
        Type result_;
        // Sum<Type, Type, detail::SubstractionOperator> sum_;
    };

}  // namespace gafro