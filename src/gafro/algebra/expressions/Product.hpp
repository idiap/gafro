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

#include <numeric>
//
#include <gafro/algebra/Blades.hpp>
#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/expressions/Product.hpp>
#include <gafro/algebra/expressions/Sum.hpp>

namespace gafro
{

    namespace detail
    {
        template <class M1, class M2, template <typename T, int, int> class Table>
        class ProductType
        {
          public:
            using Vtype = typename std::common_type<typename M1::Vtype, typename M2::Vtype>::type;

            template <int k, int... j>
            struct ProductSum;

            template <int k, int j0, int... j>
            struct ProductSum<k, j0, j...>
            {
                using Type = typename Table<Vtype, M1::blades()[k], M2::blades()[j0]>::Type;
                using Result = typename decltype(Type() + (Type() + ... + typename ProductSum<k, j>::Type()))::Type;
            };

            template <int k>
            struct ProductSum<k>
            {
                using Type = Multivector<Vtype>;
            };

            template <std::size_t... i, std::size_t... j>
            constexpr static auto makeType(std::index_sequence<i...>, std::index_sequence<j...>)
            {
                return Multivector<Vtype>() + (typename ProductSum<i, j...>::Result() + ...);
            }

            using Type = typename decltype(makeType(std::make_index_sequence<M1::size>(), std::make_index_sequence<M2::size>()))::Type;
        };

        template <class M1, class M2, template <typename T, int, int> class Table>
        class ProductValue
        {
          public:
            using Vtype = typename std::common_type<typename M1::Vtype, typename M2::Vtype>::type;

            template <int blade>
            struct Element
            {
                Element(const M1 &m1, const M2 &m2) : m1_(m1), m2_(m2) {}

                template <int k, int... j>
                struct Multiply;

                template <int k, int j0, int... j>
                struct Multiply<k, j0, j...>
                {
                    using Type = typename Table<Vtype, M1::blades()[k], M2::blades()[j0]>::Type;
                    constexpr static double sign = [] {
                        if constexpr (blade == Type::blades()[0])
                        {
                            return Table<Vtype, M1::blades()[k], M2::blades()[j0]>::signs[0];
                        }
                        else if constexpr (blade == Type::blades()[1])
                        {
                            return Table<Vtype, M1::blades()[k], M2::blades()[j0]>::signs[1];
                        }
                    }();

                    constexpr static Vtype value(const M1 &m1, const M2 &m2)
                    {
                        if constexpr (Type::has(blade))
                        {
                            return sign * m1.template get<M1::blades()[k]>() * m2.template get<M2::blades()[j0]>() +
                                   (0.0 + ... + Multiply<k, j>::value(m1, m2));
                        }
                        else
                        {
                            return (0.0 + ... + Multiply<k, j>::value(m1, m2));
                        }
                    }
                };

                template <int k>
                struct Multiply<k>
                {
                    constexpr static Vtype value(const M1 &m1, const M2 &m2)
                    {
                        return 0.0;
                    }
                };

                template <std::size_t... i, std::size_t... j>
                constexpr Vtype makeValue(std::index_sequence<i...>, std::index_sequence<j...>)
                {
                    return 0.0 + (Multiply<i, j...>::value(m1_, m2_) + ...);
                };

                constexpr Vtype value()
                {
                    return makeValue(std::make_index_sequence<M1::size>(), std::make_index_sequence<M2::size>());
                }

                const M1 &m1_;
                const M2 &m2_;
            };
        };

    }  // namespace detail

    template <class M1, class M2, template <typename T, int, int> class Table>
    class Product : public BinaryExpression<Product<M1, M2, Table>, M1, M2, typename detail::ProductType<M1, M2, Table>::Type>
    {
      public:
        using Type = typename detail::ProductType<M1, M2, Table>::Type;
        using Vtype = typename detail::ProductType<M1, M2, Table>::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;
        constexpr static auto has = Type::has;

      private:
      public:
        using Base = BinaryExpression<Product<M1, M2, Table>, M1, M2, typename detail::ProductType<M1, M2, Table>::Type>;

        Product(const Product &other) : Base(other)  //
        {}

        Product(const M1 &m1, const M2 &m2) : Base(m1, m2)  //
        {}

        template <int blade>
        requires(has(blade))  //
          Vtype get()
        const
        {
            return typename detail::ProductValue<M1, M2, Table>::template Element<blade>(this->getLeftOperand(), this->getRightOperand()).value();
        }

      private:
    };

}  // namespace gafro