// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Expression.hpp>

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