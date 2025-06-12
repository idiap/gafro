// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Expression.hpp>

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