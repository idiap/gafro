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

#include <array>
#include <list>
//
#include <gafro/algebra/Blades.hpp>
#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/expressions/GeometricProductCayleyTable.hpp>
#include <gafro/algebra/expressions/Product.hpp>
#include <gafro/algebra/expressions/Sum.hpp>

namespace gafro
{

    template <class M1, class M2>
    class GeometricProduct : public Product<M1, M2, CayleyTable>
    {
      public:
        using Product<M1, M2, CayleyTable>::Product;

        GeometricProduct(const Product<M1, M2, CayleyTable> &other) : Product<M1, M2, CayleyTable>(other)  //
        {}

        GeometricProduct(const GeometricProduct<M1, M2> &other) : Product<M1, M2, CayleyTable>(other)  //
        {}

        virtual ~GeometricProduct() = default;

        using Type = typename Product<M1, M2, CayleyTable>::Type;
        using Vtype = typename Product<M1, M2, CayleyTable>::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;

      protected:
      private:
    };

    template <class E1, class E2>
    requires(E1::isExpression() && E2::isExpression())  //
      GeometricProduct<E1, E2>
    operator*(const E1 &u, const E2 &v)
    {
        return GeometricProduct<E1, E2>(*static_cast<const E1 *>(&u), *static_cast<const E2 *>(&v));
    }

}  // namespace gafro