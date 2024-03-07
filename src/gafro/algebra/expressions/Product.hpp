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
#include <gafro/algebra/expressions/BinaryExpression.hpp>
#include <gafro/algebra/expressions/Sum.hpp>
#include <gafro/algebra/util/TypeTraits.hpp>

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
                                   (TypeTraits<Vtype>::Zero() + ... + Multiply<k, j>::value(m1, m2));
                        }
                        else
                        {
                            return (TypeTraits<Vtype>::Zero() + ... + Multiply<k, j>::value(m1, m2));
                        }
                    }
                };

                template <int k>
                struct Multiply<k>
                {
                    constexpr static Vtype value(const M1 &m1, const M2 &m2)
                    {
                        return TypeTraits<Vtype>::Zero();
                    }
                };

                template <std::size_t... i, std::size_t... j>
                constexpr static Vtype value(const M1 &m1, const M2 &m2, std::index_sequence<i...>, std::index_sequence<j...>)
                {
                    return TypeTraits<Vtype>::Zero() + (Multiply<i, j...>::value(m1, m2) + ...);
                }
            };
        };

    }  // namespace detail

    template <class M1, class M2, template <typename T, int, int> class Table>
    class Product : public BinaryExpression<Product<M1, M2, Table>, M1, M2, typename detail::ProductType<M1, M2, Table>::Type>
    {
      public:
        using Type = typename detail::ProductType<M1, M2, Table>::Type;
        using Vtype = typename detail::ProductType<M1, M2, Table>::Vtype;
        using Tensor = std::array<Eigen::Matrix<Vtype, M1::size, M2::size>, Type::size>;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;
        constexpr static auto has = Type::has;

      private:
      public:
        using Base = BinaryExpression<Product<M1, M2, Table>, M1, M2, typename detail::ProductType<M1, M2, Table>::Type>;

        Product(const M1 &m1, const M2 &m2)  //
          : Base(m1, m2)
        {}

        Product(M1 &&m1, M2 &&m2)  //
          : Base(std::move(m1), std::move(m2))
        {}

        Product(const M1 &m1, M2 &&m2)  //
          : Base(m1, std::move(m2))
        {}

        Product(M1 &&m1, const M2 &m2)  //
          : Base(std::move(m1), m2)
        {}

        template <int blade>
            requires(has(blade))  //
        Vtype get() const
        {
            return detail::ProductValue<M1, M2, Table>::template Element<blade>::value(this->getLeftOperand(),                //
                                                                                       this->getRightOperand(),               //
                                                                                       std::make_index_sequence<M1::size>(),  //
                                                                                       std::make_index_sequence<M2::size>());
        }

      private:
        class ProductTensor
        {
          public:
            using Vtype = typename std::common_type<typename M1::Vtype, typename M2::Vtype>::type;

          private:
            template <int k>
            struct Entry
            {
                template <int i, int... j>
                struct Value;

                template <int i, int j, int... js>
                struct Value<i, j, js...>
                {
                    using Blade = Table<Vtype, M1::blades()[i], M2::blades()[j]>;

                    static int add(Eigen::Matrix<Vtype, M1::size, M2::size> &entry)
                    {
                        if (!Blade::Type::blades().empty())
                        {
                            if (Type::blades()[k] == Blade::Type::blades()[0])
                            {
                                entry.coeffRef(i, j) = Blade::signs[0];
                            }
                        }

                        Value<i, js...>::add(entry);

                        return 0;
                    }
                };

                template <int i>
                struct Value<i>
                {
                    static int add(Eigen::Matrix<Vtype, M1::size, M2::size> &)
                    {
                        return 0;
                    }
                };

                template <std::size_t... i, std::size_t... j>
                static void create(Eigen::Matrix<Vtype, M1::size, M2::size> &entry, std::index_sequence<i...>, std::index_sequence<j...>)
                {
                    entry = Eigen::Matrix<Vtype, M1::size, M2::size>::Zero();

                    (Value<i, j...>::add(entry) + ...);
                }
            };

          public:
            template <int... ks>
            struct Matrix;

            template <int k, int... ks>
            struct Matrix<k, ks...>
            {
                static void create(Tensor &tensor)
                {
                    Entry<k>::create(tensor[k], std::make_index_sequence<M1::size>(), std::make_index_sequence<M2::size>());

                    Matrix<ks...>::create(tensor);
                }
            };

            template <int k>
            struct Matrix<k>
            {
                static void create(Tensor &tensor)
                {
                    Entry<k>::create(tensor[k], std::make_index_sequence<M1::size>(), std::make_index_sequence<M2::size>());
                }
            };

            template <std::size_t... k>
            static void create(Tensor &tensor, std::index_sequence<k...>)
            {
                Matrix<k...>::create(tensor);
            }
        };

      public:
        static constexpr Tensor getTensor()
        {
            Tensor tensor;

            ProductTensor::create(tensor, std::make_index_sequence<Type::size>());

            return tensor;
        }

        Eigen::Matrix<Vtype, Type::size, M1::size> getLeftJacobian() const
        {
            Eigen::Matrix<Vtype, Type::size, M1::size> left_jacobian = Eigen::Matrix<Vtype, Type::size, M1::size>::Zero();

            Tensor tensor = getTensor();

            for (unsigned k = 0; k < Type::size; ++k)
            {
                for (unsigned j = 0; j < M2::size; ++j)
                {
                    left_jacobian.row(k) += this->getRightOperand().vector()[j] * tensor[k].col(j).transpose();
                }
            }

            return left_jacobian;
        }

        Eigen::Matrix<Vtype, Type::size, M2::size> getRightJacobian() const
        {
            Eigen::Matrix<Vtype, Type::size, M2::size> right_jacobian = Eigen::Matrix<Vtype, Type::size, M2::size>::Zero();

            Tensor tensor = getTensor();

            for (unsigned k = 0; k < Type::size; ++k)
            {
                for (unsigned i = 0; i < M1::size; ++i)
                {
                    right_jacobian.row(k) += this->getLeftOperand().vector()[i] * tensor[k].row(i);
                }
            }

            return right_jacobian;
        }
    };

}  // namespace gafro