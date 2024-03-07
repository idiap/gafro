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

#include <iostream>
#include <ostream>
//
#include <gafro/algebra/expressions/AbstractExpression.hpp>

namespace gafro
{

    template <class Derived, class Result>
    class Expression : public AbstractExpression<Derived>
    {
      public:
        Expression() = default;

        virtual ~Expression() = default;

        using Type = Result;
        using Vtype = typename Type::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;

        template <int blade>
            requires(Type::has(blade))  //
        typename Result::Vtype get() const
        {
            return static_cast<const Derived &>(*this).template get<blade>();
        }

        template <int blade>
            requires(Type::has(blade))  //
        constexpr static int map()
        {
            return Type::template map<blade>();
        }

        constexpr static int has(const int &blade)
        {
            return Type::has(blade);
        }

        Result evaluate() const
        {
            return Result(*this);
        }

        typename Result::Parameters vector() const
        {
            return evaluate().vector();
        }

        template <class R>
        R evaluateAs() const
        {
            return R(*this);
        }

      protected:
      private:
    };

    template <class Derived, class Result>
    std::ostream &operator<<(std::ostream &ostream, const Expression<Derived, Result> &expression)
    {
        ostream << expression.evaluate();

        return ostream;
    }

}  // namespace gafro