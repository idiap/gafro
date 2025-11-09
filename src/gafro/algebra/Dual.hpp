// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/GeometricProduct.hpp>
#include <gafro/algebra/UnaryExpression.hpp>

namespace gafro
{

    template <class M>
    class Dual
      : public UnaryExpression<Dual<M>, M, typename GeometricProduct<M, typename M::MAlgebra::template Pseudoscalar<typename M::Vtype>>::Type>
    {
      public:
        using Base = UnaryExpression<Dual<M>, M, typename GeometricProduct<M, typename M::MAlgebra::template Pseudoscalar<typename M::Vtype>>::Type>;

        using Vtype = typename M::Vtype;

        using Type = typename GeometricProduct<M, typename M::MAlgebra::template Pseudoscalar<typename M::Vtype>>::Type;

        constexpr static int  size   = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits   = Type::bits;
        constexpr static auto has    = Type::has;

        Dual(const M &m1)
          : Base(m1)
        {}

        Dual(M &&m1)
          : Base(std::move(m1))
        {}

        virtual ~Dual() = default;

        template <int blade>
            requires(has(blade))  //
        Vtype get() const
        {
            return (this->operand() * typename M::MAlgebra::template Pseudoscalar<Vtype>(Vtype(-1.0))).template get<blade>();
        }

      protected:
      private:
    };

}  // namespace gafro