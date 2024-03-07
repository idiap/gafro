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
#include <gafro/algebra/AbstractMultivector.hpp>
#include <gafro/algebra/util/Bitset.hpp>

namespace gafro
{
    template <class Derived, class Result>
    class Expression;

    template <class M>
    class Reverse;

    template <class M>
    class Inverse;

    template <class M>
    class Dual;

    template <class M1, class M2>
    class CommutatorProduct;

    template <class T, template <class S> class M, int rows, int cols>
    class MultivectorMatrix;

    template <class T, int... index>
    class Multivector : public AbstractMultivector<Multivector<T, index...>>
    {
      public:
        constexpr static int size = Bitset<index...>().size();

        using Vtype = typename std::remove_const<T>::type;

        using Type = Multivector;

        using Parameters = Eigen::Matrix<T, size, 1>;

        //

        Multivector();

        Multivector(const int &value);

        Multivector(const Parameters &parameters);

        Multivector(Parameters &&parameters);

        Multivector(const Multivector &other);

        Multivector(Multivector &&other);

        template <class Derived>
        Multivector(const Expression<Derived, Multivector> &expression);

        template <class Derived, class Other>
        Multivector(const Expression<Derived, Other> &expression);

        template <class S>
        Multivector(const Multivector<S, index...> &other);

        //

        virtual ~Multivector() = default;

        //

        Multivector &operator=(const Parameters &parameters);

        Multivector &operator=(Parameters &&parameters);

        Multivector &operator=(const Multivector &other);

        Multivector &operator=(Multivector &&other);

        Multivector &operator*=(const T &scalar);

        Multivector &operator/=(const T &scalar);

        Multivector &operator+=(const Multivector &other);

        template <class Derived>
        Multivector &operator=(const Expression<Derived, Multivector> &expression);

        template <class Derived, class Other>
        Multivector &operator=(const Expression<Derived, Other> &expression);

        //

        void setParameters(Parameters &&parameters);

        void setParameters(const Parameters &parameters);

        //

        Parameters &vector();

        const Parameters &vector() const;

        //

        Reverse<Multivector> reverse() const;

        Inverse<Multivector> inverse() const;

        Dual<Multivector> dual() const;

        //

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

      public:
        template <int i>
        constexpr static int map()
        {
            return Map<i, index...>::idx;
        }

        template <int blade>
            requires(has(blade))  //
        void set(const T &value)
        {
            parameters_.coeffRef(map<blade>()) = value;
        }

        template <int blade>
            requires(has(blade))  //
        const T &get() const
        {
            return parameters_.coeff(map<blade>());
        }

      public:
        //

        T norm() const;

        T squaredNorm() const;

        T signedNorm() const;

        void normalize();

        Multivector normalized() const;

        //

        template <class Other>
        Other cast() const;

        template <class M2>
        CommutatorProduct<Multivector, M2> commutatorProduct(const M2 &multivector) const;

        //

        template <class t>
        using M = Multivector<t, index...>;

        template <int rows, int cols>
        using Matrix = MultivectorMatrix<T, M, rows, cols>;

        template <int rows, int cols>
        static Matrix<rows, cols> CreateMatrix()
        {
            return MultivectorMatrix<T, M, rows, cols>();
        }

      public:
        auto getBlades() const;

      private:
        template <std::size_t... i>
        constexpr static auto getBlades(std::index_sequence<i...>, const Multivector &multivector);

        //

      public:
        static Multivector Random();

        static Multivector Zero();

      private:
        Parameters parameters_;

        constexpr static Bitset<index...> bits_ = Bitset<index...>();

        constexpr static const std::array<int, size> blades_ = bits_.blades();
    };

}  // namespace gafro