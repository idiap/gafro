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
#include <gafro/algebra/expressions/UnaryExpression.hpp>

namespace gafro
{

    template <class M>
    class Reverse : public UnaryExpression<Reverse<M>, M, typename M::Type>
    {
      public:
        using Type = typename M::Type;
        using Vtype = typename M::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;
        // constexpr static auto map = Type::map;
        constexpr static auto has = Type::has;

        Reverse(const M &m1)  //
          : UnaryExpression<Reverse<M>, M, typename M::Type>(m1)
        {}

        Reverse(M &&m1)  //
          : UnaryExpression<Reverse<M>, M, typename M::Type>(std::move(m1))
        {}

        virtual ~Reverse() = default;

        template <int blade>
            requires(has(blade))  //
        Vtype get() const
        {
            if constexpr (blade > 5 && blade < 26)
            {
                return -this->operand().template get<blade>();
            }
            else
            {
                return this->operand().template get<blade>();
            }
        }

      protected:
      private:
    };

}  // namespace gafro