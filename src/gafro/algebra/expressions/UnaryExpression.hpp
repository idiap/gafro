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

#include <gafro/algebra/expressions/Expression.hpp>

namespace gafro
{
    template <class Derived, class Operand, class Result>
    class UnaryExpression : public Expression<UnaryExpression<Derived, Operand, Result>, Result>
    {
      public:
        UnaryExpression(const Operand &operand)  //
          : operand_(operand)
        {}

        UnaryExpression(Operand &&operand)  //
          : operand_(std::move(operand))
        {}

        virtual ~UnaryExpression() = default;

        using Type = Result;

        template <int blade>
            requires(Type::has(blade))  //
        typename Result::Vtype get() const
        {
            return static_cast<const Derived *>(this)->template get<blade>();
        }

      protected:
        const Operand &operand() const
        {
            return operand_;
        }

      private:
        const Operand operand_;
    };
}  // namespace gafro