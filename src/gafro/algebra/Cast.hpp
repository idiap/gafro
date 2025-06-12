// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/UnaryExpression.hpp>

namespace gafro
{

    template <class M, class Result>
    class Cast : public UnaryExpression<Cast<M, Result>, M, Result>
    {
      public:
        using Type = typename M::Type;
        using Vtype = typename M::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;
        constexpr static auto has = Type::has;

        Cast(const M &m1) : UnaryExpression<Cast<M, Result>, M, Result>(m1) {}

        virtual ~Cast() = default;

        template <int blade>
            requires(has(blade))  //
        Vtype get() const
        {
            return this->operand().template get<blade>();
        }

      protected:
      private:
    };

}  // namespace gafro