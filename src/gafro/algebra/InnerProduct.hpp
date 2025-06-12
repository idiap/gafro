// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <array>
#include <list>
//
#include <gafro/algebra/Multivector.hxx>
// #include <gafro/algebra/InnerProductCayleyTable.hpp>
#include <gafro/algebra/Product.hpp>
#include <gafro/algebra/Sum.hpp>

namespace gafro
{

    template <class M1, class M2>
    class InnerProduct : public Product<M1, M2, Algebra<typename M1::Metric>::template InnerProduct>
    {
      public:
        InnerProduct(const M1 &m1, const M2 &m2)  //
          : Product<M1, M2, Algebra<typename M1::Metric>::template InnerProduct>(m1, m2)
        {}

        InnerProduct(M1 &&m1, M2 &&m2)  //
          : Product<M1, M2, Algebra<typename M1::Metric>::template InnerProduct>(std::move(m1), std::move(m2))
        {}

        InnerProduct(const M1 &m1, M2 &&m2)  //
          : Product<M1, M2, Algebra<typename M1::Metric>::template InnerProduct>(m1, std::move(m2))
        {}

        InnerProduct(M1 &&m1, const M2 &m2)  //
          : Product<M1, M2, Algebra<typename M1::Metric>::template InnerProduct>(std::move(m1), m2)
        {}

        virtual ~InnerProduct() = default;

        using Type = typename Product<M1, M2, Algebra<typename M1::Metric>::template InnerProduct>::Type;
        using Vtype = typename Product<M1, M2, Algebra<typename M1::Metric>::template InnerProduct>::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;

      protected:
      private:
    };

}  // namespace gafro