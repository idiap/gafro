// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/BinaryExpression.hpp>
#include <gafro/algebra/GeometricProduct.hpp>

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