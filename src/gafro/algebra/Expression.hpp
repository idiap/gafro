// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <iostream>
#include <ostream>
//
#include <gafro/algebra/AbstractExpression.hpp>

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

        Type operator-() const
        {
            return -evaluate();
        }

        Type operator*(const Vtype &scalar) const
        {
            return scalar * evaluate();
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

    template <class Derived, class Result>
    Result operator*(const typename Result::Vtype &scalar, const Expression<Derived, Result> &expression)
    {
        return scalar * expression.evaluate();
    }

}  // namespace gafro