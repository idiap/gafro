// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <bitset>
#include <iostream>
#include <map>
#include <memory>
//
#include <Eigen/Core>
//
#include <gafro/algebra/AbstractMultivector.hpp>
#include <gafro/algebra/Algebra.hpp>
#include <gafro/algebra/Bitset.hpp>

namespace gafro
{
    template <class Derived, class Result>
    class Expression;

    template <class M>
    class Reverse;

    template <class M>
    class Conjugate;

    template <class M>
    class SharpConjugate;

    template <class M>
    class Inverse;

    template <class M>
    class Dual;

    template <class M>
    class DualPoincare;

    template <class M1, class M2>
    class CommutatorProduct;

    template <class T, template <class S> class M, int rows, int cols>
    class MultivectorMatrix;

    namespace probabilistic
    {
        template <class T, template <class S> class M>
        class MultivectorGaussian;
    }

    template <class M>
    template <class T, int... index>
    class Algebra<M>::Multivector : public AbstractMultivector<Multivector<T, index...>>
    {
      public:
        using Vtype = typename std::remove_const<T>::type;

        using Type = Multivector;

        using MAlgebra = Algebra<M>;

        using Metric = M;

        constexpr static int size = Bitset<MAlgebra::dim, index...>().size();

        using Parameters = Eigen::Matrix<T, size, 1>;

        using PoincareDual = Algebra<M>::Multivector<T, MAlgebra::BladeBitmap::template getPoincareDual<index>()...>;

        //

        Multivector();

        Multivector(const T &value)
            requires(sizeof...(index) == 1);

        Multivector(const Parameters &parameters);

        Multivector(Parameters &&parameters);

        Multivector(const Multivector &other);

        Multivector(Multivector &&other);

        template <class Derived>
        Multivector(const Expression<Derived, Multivector> &expression);

        template <class Derived, class Other>
        Multivector(const Expression<Derived, Other> &expression);

        template <class S>
        Multivector(const typename Algebra<M>::template Multivector<S, index...> &other);

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

        Multivector operator-() const;

        Multivector operator*(const T &scalar) const;

        Multivector operator/(const T &scalar) const;

        template <template <class S> class MType, int rows, int cols>
        auto operator*(const MultivectorMatrix<T, MType, rows, cols> &matrix) const;

        template <template <class S> class MType, int rows, int cols>
        auto operator|(const MultivectorMatrix<T, MType, rows, cols> &matrix) const;

        template <template <class S> class MType, int rows, int cols>
        auto operator^(const MultivectorMatrix<T, MType, rows, cols> &matrix) const;

        template <template <class S> class MType, int rows, int cols>
        auto operator+(const MultivectorMatrix<T, MType, rows, cols> &matrix) const;

        template <template <class S> class MType, int rows, int cols>
        auto operator-(const MultivectorMatrix<T, MType, rows, cols> &matrix) const;

        //

        void setParameters(Parameters &&parameters);

        void setParameters(const Parameters &parameters);

        //

        Parameters &vector();

        const Parameters &vector() const;

        //

        Reverse<Multivector> reverse() const;

        Conjugate<Multivector> conjugate() const;

        SharpConjugate<Multivector> sharpConjugate() const;

        Inverse<Multivector> inverse() const;

        Dual<Multivector> dual() const;

        DualPoincare<Multivector> dualPoincare() const;

        //

        constexpr static const Bitset<MAlgebra::dim, index...> &bits();

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

        template <int blade>
            requires(has(blade))  //
        Multivector<T, blade> extract() const
        {
            return Multivector<T, blade>(get<blade>());
        }

        template <int blade>
            requires(has(blade))  //
        Multivector<T, blade> getBlade() const
        {
            return typename Algebra<M>::template Multivector<T, blade>((Eigen::Matrix<T, 1, 1>() << this->template get<blade>()).finished());
        }

      public:
        //

        T norm() const;

        T squaredNorm() const;

        T signedNorm() const;

        void normalize();

        Multivector normalized() const;

        //

        auto square() const;

        //

        template <class Other>
        Other cast() const;

        template <class M2>
        auto commute(const M2 &multivector) const;

        template <class M2>
        auto anticommute(const M2 &multivector) const;

        //

        template <class t>
        using MV = Multivector<t, index...>;

        template <int rows, int cols>
        using Matrix = MultivectorMatrix<T, MV, rows, cols>;

        using Gaussian = probabilistic::MultivectorGaussian<T, MV>;

        //

      public:
        auto getBlades() const;

        constexpr static int getGrade()
        {
            std::array<int, sizeof...(index)> blades = { MAlgebra::BladeBitmap::template getGrade<index>()... };

            return *std::max_element(blades.begin(), blades.end());
        }

      private:
        template <std::size_t... i>
        constexpr static auto getBlades(std::index_sequence<i...>, const Multivector &multivector);

        //

      public:
        static Multivector Random();

        static Multivector One();

        static Multivector Zero();

      private:
        Parameters parameters_;

        constexpr static Bitset<MAlgebra::dim, index...> bits_ = Bitset<MAlgebra::dim, index...>();

        constexpr static const std::array<int, size> blades_ = bits_.blades();

      public:
        void print() const
        {
            bool first = true;

            for (unsigned int k = 0; k < vector().rows(); ++k)
            {
                if (abs(vector().coeff(k, 0)) < 1e-10)
                {
                    continue;
                }

                if (!first)
                {
                    std::cout << (vector().coeff(k, 0) >= 0 ? " + " : " - ");

                    std::cout << abs(vector().coeff(k, 0));
                }
                else
                {
                    std::cout << vector().coeff(k, 0);

                    first = false;
                }

                if (blades()[k] > 0)
                {
                    std::string blade_name = "e";
                    for (int j = 0; j < M::dim; ++j)
                    {
                        if (((1 << j) & blades()[k]) != 0)
                        {
                            blade_name += std::to_string(j + 1);
                        }
                    }
                    std::cout << "*" << blade_name;
                }
            }

            if (first)
            {
                std::cout << 0;
            }

            std::cout << std::endl;
        }
    };

}  // namespace gafro