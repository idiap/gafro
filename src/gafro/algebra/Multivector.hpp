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

#include <bitset>
#include <iostream>
#include <map>
#include <memory>
//
#include <Eigen/Core>
//
#include <gafro/algebra/util/Bitset.hpp>

namespace gafro
{
    template <class M>
    class Reverse;

    template <class M>
    class Inverse;

    template <class M>
    class Dual;

    template <class T, int... index>
    class Multivector
    {
      public:
        constexpr static int size = Bitset<index...>().size();

        using Vtype = typename std::remove_const<T>::type;

        using Type = Multivector;

        using Parameters = Eigen::Matrix<T, size, 1>;

        constexpr static bool isExpression()
        {
            return true;
        }

        Multivector();

        Multivector(const int &value);

        Multivector(const Parameters &parameters);

        Multivector(Multivector &&other);

        Multivector(const Multivector &other);

        virtual ~Multivector() = default;

        void setParameters(Parameters &&parameters);

        void setParameters(const Parameters &parameters);

        Parameters &vector();

        const Parameters &vector() const;

        constexpr static const Bitset<index...> &bits();

        constexpr static const std::array<int, size> &blades();

        constexpr static bool has(const int &blade);

      private:
        template <int i, int... j>
        struct Map;

        template <int i, int j0, int... j>
        struct Map<i, j0, j...>
        {
            constexpr static int idx = [] {
                if constexpr (i == j0)
                {
                    return sizeof...(index) - sizeof...(j) - 1;
                }
                else
                {
                    return Map<i, j...>::idx;
                }
            }();
        };

        template <int i, int j>
        struct Map<i, j>
        {
            constexpr static int idx = sizeof...(index) - 1;
        };

        template <int i>
        constexpr static int map()
        {
            return Map<i, index...>::idx;
        }

      public:
        Reverse<Multivector> reverse() const;

        Inverse<Multivector> inverse() const;

        Dual<Multivector> dual() const;

        T norm() const;

        T squaredNorm() const;

        T signedNorm() const;

        void normalize();

        template <int blade>
        requires(has(blade))  //
          void set(const T &value)
        {
            parameters_.coeffRef(map<blade>(), 0) = value;
        }

        template <int blade>
        requires(has(blade))  //
          const T &get() const
        {
            return parameters_.coeff(map<blade>(), 0);
        }

        const T &operator[](int blade) const;

        template <class Other>
        Other cast() const;

        template <std::size_t... i>
        constexpr static auto getBlades(std::index_sequence<i...>);

        constexpr static auto split();

        Multivector &operator+=(const Multivector &other)
        {
            parameters_ += other.parameters_;

            return *this;
        }

      private:
        template <class Expression, int... j>
        struct Assign;

        template <class Expression, int k, int... j>
        struct Assign<Expression, k, j...>
        {
            static void run(const Expression &expression, Parameters &parameters)
            {
                parameters.coeffRef(k, 0) = expression.template get<blades()[k]>();

                Assign<Expression, j...>::run(expression, parameters);
            }
        };

        template <class Expression, int k>
        struct Assign<Expression, k>
        {
            static void run(const Expression &expression, Parameters &parameters)
            {
                parameters.coeffRef(k, 0) = expression.template get<blades()[k]>();
            }
        };

        template <class Expression, std::size_t... i>
        constexpr static void assign(const Expression &expression, Parameters &parameters, std::index_sequence<i...>)
        {
            Assign<Expression, i...>::run(expression, parameters);
        };

      public:
        template <class Expression>
        requires(Expression::isExpression())  //
          Multivector(const Expression &expression)
        {
            assign(expression, parameters_, std::make_index_sequence<size>());
        }

        template <class Expression>
        requires(Expression::isExpression())  //
          Multivector &
          operator=(const Expression &expression)
        {
            assign(expression, parameters_, std::make_index_sequence<size>());

            return *this;
        }

        Multivector &operator=(const Multivector &other)
        {
            setParameters(other.vector());

            return *this;
        }

        Multivector &operator=(Multivector &&other)
        {
            setParameters(std::forward<Parameters>(other.vector()));

            return *this;
        }

      public:
        static Multivector Random();

      private:
        Parameters parameters_;

        constexpr static Bitset<index...> bits_ = Bitset<index...>();

        constexpr static const std::array<int, size> blades_ = bits_.blades();

        // constexpr static const std::array<int, 32> map_ = [] {
        //     std::array<int, 32> array;

        //     int i = 0;

        //     for (unsigned k = 0; k < 32; ++k)
        //     {
        //         if (has(k))
        //         {
        //             array[k] = i++;
        //         }
        //         else
        //         {
        //             array[k] = -1;
        //         }
        //     }

        //     return array;
        // }();
    };

}  // namespace gafro