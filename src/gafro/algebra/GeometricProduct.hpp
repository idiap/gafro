// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <array>
#include <list>
//
#include <gafro/algebra/Algebra.hpp>
#include <gafro/algebra/Product.hpp>

namespace gafro
{

    template <class M1, class M2>
    class GeometricProduct : public Product<M1, M2, Algebra<typename M1::Metric>::template GeometricProduct>
    {
      public:
        GeometricProduct(const M1 &m1, const M2 &m2)  //
          : Product<M1, M2, Algebra<typename M1::Metric>::template GeometricProduct>(m1, m2)
        {}

        GeometricProduct(M1 &&m1, M2 &&m2)  //
          : Product<M1, M2, Algebra<typename M1::Metric>::template GeometricProduct>(std::move(m1), std::move(m2))
        {}

        GeometricProduct(const M1 &m1, M2 &&m2)  //
          : Product<M1, M2, Algebra<typename M1::Metric>::template GeometricProduct>(m1, std::move(m2))
        {}

        GeometricProduct(M1 &&m1, const M2 &m2)  //
          : Product<M1, M2, Algebra<typename M1::Metric>::template GeometricProduct>(std::move(m1), m2)
        {}

        virtual ~GeometricProduct() = default;

        using Type = typename Product<M1, M2, Algebra<typename M1::Metric>::template GeometricProduct>::Type;
        using Vtype = typename Product<M1, M2, Algebra<typename M1::Metric>::template GeometricProduct>::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;

      protected:
      private:
    };

}  // namespace gafro