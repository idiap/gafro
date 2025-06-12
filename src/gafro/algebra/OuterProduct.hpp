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
#include <gafro/algebra/OuterProductCayleyTable.hpp>
#include <gafro/algebra/Product.hpp>
#include <gafro/algebra/Sum.hpp>

namespace gafro
{

    template <class M1, class M2>
    class OuterProduct : public Product<M1, M2, Algebra<typename M1::Metric>::template OuterProduct>
    {
      public:
        OuterProduct(const M1 &m1, const M2 &m2) : Product<M1, M2, Algebra<typename M1::Metric>::template OuterProduct>(m1, m2) {}

        OuterProduct(const Product<M1, M2, Algebra<typename M1::Metric>::template OuterProduct> &other) {}

        virtual ~OuterProduct() = default;

        using Type = typename Product<M1, M2, Algebra<typename M1::Metric>::template OuterProduct>::Type;
        using Vtype = typename Product<M1, M2, Algebra<typename M1::Metric>::template OuterProduct>::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;

      protected:
      private:
    };

}  // namespace gafro