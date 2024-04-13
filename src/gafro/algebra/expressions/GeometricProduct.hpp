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
#include <gafro/algebra/Algebra.hpp>
#include <gafro/algebra/expressions/Product.hpp>

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