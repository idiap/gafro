// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Multivector.hxx>

namespace gafro
{
    template <class T>
    class Motor;

    template <class T, template <class S> class M, int rows, int cols>
    class MultivectorMatrix
    {
      public:
        using Type = M<T>;
        using Matrix = Eigen::Matrix<T, rows * Type::size, cols>;

        MultivectorMatrix();

        MultivectorMatrix(const Matrix &parameters);

        virtual ~MultivectorMatrix();

        Type &getCoefficient(int row, int col);

        const Type &getCoefficient(int row, int col) const;

        void setCoefficient(int row, int col, const Type &multivector);

        Matrix embed() const;

        MultivectorMatrix normalized() const;

        MultivectorMatrix reverse() const;

        MultivectorMatrix inverse() const;

        auto dual() const;

        MultivectorMatrix transform(const Motor<T> &motor) const;

        template <template <class S2> class M2>
        MultivectorMatrix<T, M2, rows, cols> extract() const;

        std::vector<M<T>> asVector() const;

      protected:
      private:
        std::array<std::array<Type, cols>, rows> multivectors_;

      public:
        MultivectorMatrix &operator+=(const Type &multivector);

        MultivectorMatrix &operator*=(const Type &multivector);

        template <int... blades>
        auto operator*(const typename Type::MAlgebra::template Multivector<T, blades...> &multivector) const;

        template <int... blades>
        auto operator^(const typename Type::MAlgebra::template Multivector<T, blades...> &multivector) const;

        template <int... blades>
        auto operator|(const typename Type::MAlgebra::template Multivector<T, blades...> &multivector) const;

        template <template <class S2> class M2, int rows2, int cols2>
        auto operator*(const MultivectorMatrix<T, M2, rows2, cols2> &matrix) const;

        template <template <class S2> class M2, int rows2, int cols2>
        auto operator|(const MultivectorMatrix<T, M2, rows2, cols2> &matrix) const;

        template <template <class S2> class M2>
        auto operator+(const MultivectorMatrix<T, M2, rows, cols> &matrix) const;

        template <template <class S2> class M2>
        auto operator-(const MultivectorMatrix<T, M2, rows, cols> &matrix) const;

        // MultivectorMatrix operator|(const MultivectorMatrix &matrix) const;

        // MultivectorMatrix operator+(const MultivectorMatrix &matrix) const;
    };

}  // namespace gafro

#include <gafro/algebra/MultivectorMatrix.hxx>