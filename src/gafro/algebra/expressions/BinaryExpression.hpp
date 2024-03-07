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

    template <class Derived, class LeftOperand, class RightOperand, class Result>
    class BinaryExpression : public Expression<BinaryExpression<Derived, LeftOperand, RightOperand, Result>, Result>
    {
      public:
        BinaryExpression(const LeftOperand &left_operand, const RightOperand &right_operand)
          : left_operand_(left_operand), right_operand_(right_operand)
        {}

        BinaryExpression(LeftOperand &&left_operand, RightOperand &&right_operand)
          : left_operand_(std::move(left_operand)), right_operand_(std::move(right_operand))
        {}

        BinaryExpression(const LeftOperand &left_operand, RightOperand &&right_operand)
          : left_operand_(left_operand), right_operand_(std::move(right_operand))
        {}

        BinaryExpression(LeftOperand &&left_operand, const RightOperand &right_operand)
          : left_operand_(std::move(left_operand)), right_operand_(right_operand)
        {}

        virtual ~BinaryExpression() = default;

        using Type = Result;
        using Vtype = typename Result::Vtype;

        template <int blade>
            requires(Type::has(blade))  //
        Vtype get() const
        {
            return static_cast<const Derived &>(*this).template get<blade>();
        }

        const LeftOperand &getLeftOperand() const
        {
            return left_operand_;
        }

        const RightOperand &getRightOperand() const
        {
            return right_operand_;
        }

      protected:
      private:
        const LeftOperand left_operand_;
        const RightOperand right_operand_;
    };
}  // namespace gafro