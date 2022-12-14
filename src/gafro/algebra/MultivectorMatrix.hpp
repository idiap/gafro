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

#include <gafro/algebra/Multivector.hxx>

// enable support for matrices of multivectors
namespace Eigen
{
    template <class T, int... index>
    struct NumTraits<gafro::Multivector<T, index...>> : NumTraits<T>  // permits to get the epsilon, dummy_precision, lowest, highest functions
    {
        typedef gafro::Multivector<T, index...> Real;
        typedef gafro::Multivector<T, index...> NonInteger;
        typedef gafro::Multivector<T, index...> Nested;

        enum
        {
            IsComplex = 0,
            IsInteger = 0,
            IsSigned = 1,
            RequireInitialization = 1,
            ReadCost = 1,
            AddCost = 3,
            MulCost = 3
        };
    };

    template <class T>
    struct NumTraits<gafro::Scalar<T>> : NumTraits<T>  // permits to get the epsilon, dummy_precision, lowest, highest functions
    {
        typedef gafro::Scalar<T> Real;
        typedef gafro::Scalar<T> NonInteger;
        typedef gafro::Scalar<T> Nested;

        enum
        {
            IsComplex = 0,
            IsInteger = 0,
            IsSigned = 1,
            RequireInitialization = 1,
            ReadCost = 1,
            AddCost = 3,
            MulCost = 3
        };
    };

    namespace internal
    {
        // template <int rows, int cols, class T, int... index>
        // struct traits<Matrix<gafro::Multivector<T, index...>, rows, cols>>
        // {
        //     typedef Eigen::Dense StorageKind;
        //     typedef Eigen::MatrixXpr XprKind;
        //     typedef gafro::Multivector<T, index...> Scalar;
        //     typedef typename Matrix<T, rows, cols>::StorageIndex StorageIndex;
        //     enum
        //     {
        //         Flags = Eigen::ColMajor | Eigen::LvalueBit,
        //         RowsAtCompileTime = rows,
        //         ColsAtCompileTime = cols,
        //         MaxRowsAtCompileTime = rows,
        //         MaxColsAtCompileTime = cols,
        //         Options = internal::traits<Matrix<T, rows, cols>>::Options,
        //         Alignment = internal::traits<Matrix<T, rows, cols>>::Alignment,
        //         EvaluatorFlags = internal::traits<Matrix<T, rows, cols>>::EvaluatorFlags
        //     };
        // };

        // template <class M1, int r1, int c1, class M2, int r2, int c2>
        // struct traits<Eigen::Product<Matrix<M1, r1, c1>, Matrix<M2, r2, c2>, 0>>
        // {
        //     typedef Eigen::Dense StorageKind;
        //     typedef Eigen::MatrixXpr XprKind;
        //     typedef typename gafro::GeometricProduct<M1, M2>::Type Scalar;
        //     typedef typename Matrix<Scalar, r1, c2>::StorageIndex StorageIndex;
        //     enum
        //     {
        //         Flags = Eigen::ColMajor,
        //         RowsAtCompileTime = r1,
        //         ColsAtCompileTime = c2,
        //         MaxRowsAtCompileTime = r1,
        //         MaxColsAtCompileTime = c2
        //     };
        // };
    }  // namespace internal

    template <class T, int... i1, int... i2>
    struct ScalarBinaryOpTraits<gafro::Multivector<T, i1...>, gafro::Multivector<T, i2...>,
                                Eigen::internal::scalar_product_op<gafro::Multivector<T, i1...>, gafro::Multivector<T, i2...>>>
    {
        using ReturnType = typename gafro::GeometricProduct<gafro::Multivector<T, i1...>, gafro::Multivector<T, i2...>>::Type;
    };

    template <class T, int... i>
    struct ScalarBinaryOpTraits<gafro::Multivector<T, i...>, gafro::Multivector<T, i...>,
                                Eigen::internal::scalar_product_op<gafro::Multivector<T, i...>, gafro::Multivector<T, i...>>>
    {
        using ReturnType = typename gafro::GeometricProduct<gafro::Multivector<T, i...>, gafro::Multivector<T, i...>>::Type;
    };

}  // namespace Eigen

namespace gafro
{
    template <class M, int rows, int cols>
    class MultivectorMatrix : public Eigen::Matrix<M, rows, cols>
    {
      public:
        using Base = Eigen::Matrix<M, rows, cols>;
        using Eigen::Matrix<M, rows, cols>::Matrix;
        using Eigen::Matrix<M, rows, cols>::operator=;
        using Eigen::Matrix<M, rows, cols>::operator-;
        using Eigen::Matrix<M, rows, cols>::operator+;
        using Eigen::Matrix<M, rows, cols>::operator*;
        using T = typename M::Vtype;

      public:
        MultivectorMatrix();

        template <typename OtherDerived>
        MultivectorMatrix(const Eigen::MatrixBase<OtherDerived> &other);

        template <class Other>
        void assign(const Eigen::MatrixBase<Other> &other)
        {
            for (int row = 0; row < rows; ++row)
            {
                for (int col = 0; col < cols; ++col)
                {
                    this->coeffRef(row, col) = other.coeff(row, col).template cast<M>();
                }
            }
        }

        virtual ~MultivectorMatrix();

        template <class Other>
        MultivectorMatrix<typename GeometricProduct<M, Other>::Type, rows, cols> rmult(const Other &other) const
        {
            return this->unaryExpr([&other](const M &m) { return (m * other).evaluate(); });
        }

        template <class Other>
        MultivectorMatrix<typename GeometricProduct<Other, M>::Type, rows, cols> lmult(const Other &other) const
        {
            return this->unaryExpr([&other](const M &m) { return (other * m).evaluate(); });
        }

        Eigen::Matrix<T, rows * M::size, cols> embed() const;

        template <class M2>
        Eigen::Matrix<T, rows * M2::size, cols> embedAs() const;

        template <class M2>
        // requires(M::has(M2::index))  //
        MultivectorMatrix<M2, rows, cols> extract() const;

        MultivectorMatrix reverse() const;

      protected:
      private:
      public:
        template <typename OtherDerived>
        MultivectorMatrix &operator+=(const Eigen::MatrixBase<OtherDerived> &other);

        template <typename OtherDerived>
        MultivectorMatrix &operator=(const Eigen::MatrixBase<OtherDerived> &other);
    };

    template <class M, int rows, int cols>
    MultivectorMatrix<M, rows, cols>::MultivectorMatrix()
    {}

    template <class M, int rows, int cols>
    template <typename OtherDerived>
    MultivectorMatrix<M, rows, cols>::MultivectorMatrix(const Eigen::MatrixBase<OtherDerived> &other) : Eigen::Matrix<M, rows, cols>(other)
    {}

    template <class M, int rows, int cols>
    MultivectorMatrix<M, rows, cols>::~MultivectorMatrix()
    {}

    template <class M, int rows, int cols>
    template <typename OtherDerived>
    MultivectorMatrix<M, rows, cols> &MultivectorMatrix<M, rows, cols>::operator=(const Eigen::MatrixBase<OtherDerived> &other)
    {
        this->Eigen::Matrix<M, rows, cols>::operator=(other.template cast<M>());

        return *this;
    }

    template <class M, int rows, int cols>
    template <typename OtherDerived>
    MultivectorMatrix<M, rows, cols> &MultivectorMatrix<M, rows, cols>::operator+=(const Eigen::MatrixBase<OtherDerived> &other)
    {
        this->Eigen::Matrix<M, rows, cols>::operator+=(other.template cast<M>());

        return *this;
    }

    template <class M, int rows, int cols>
    Eigen::Matrix<typename M::Vtype, rows * M::size, cols> MultivectorMatrix<M, rows, cols>::embed() const
    {
        Eigen::Matrix<typename M::Vtype, rows * M::size, cols> embedding = Eigen::Matrix<typename M::Vtype, rows * M::size, cols>::Zero();

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                embedding.block(r * M::size, c, M::size, 1) = this->coeff(r, c).vector();
            }
        }

        return embedding;
    }

    template <class M, int rows, int cols>
    template <class M2>
    Eigen::Matrix<typename M::Vtype, rows * M2::size, cols> MultivectorMatrix<M, rows, cols>::embedAs() const
    {
        Eigen::Matrix<typename M2::Vtype, rows * M2::size, cols> embedding = Eigen::Matrix<typename M2::Vtype, rows * M2::size, cols>::Zero();

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                embedding.block(r * M2::size, c, M2::size, 1) = this->coeff(r, c).template cast<M2>().vector();
            }
        }

        return embedding;
    }

    template <class M, int rows, int cols>
    template <class M2>
    // requires(M::has(M2::index))  //
    MultivectorMatrix<M2, rows, cols> MultivectorMatrix<M, rows, cols>::extract() const
    {
        MultivectorMatrix<M2, rows, cols> extracted;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                extracted.coeffRef(r, c) = this->coeff(r, c).template cast<M2>();
            }
        }

        return extracted;
    }

    template <class M, int rows, int cols>
    MultivectorMatrix<M, rows, cols> MultivectorMatrix<M, rows, cols>::reverse() const
    {
        return this->unaryExpr([](const M &m) { return m.reverse().evaluate(); });
    }

    template <class M>
    using MultivectorMatrixX = MultivectorMatrix<M, Eigen::Dynamic, Eigen::Dynamic>;

}  // namespace gafro

template <class M, int rows, int cols>
std::ostream &operator<<(std::ostream &ostream, const gafro::MultivectorMatrix<M, rows, cols> &matrix)
{
    ostream << std::endl;
    for (unsigned r = 0; r < rows; r++)
    {
        for (unsigned c = 0; c < cols; c++)
        {
            std::cout << r << "," << c << ":\t";
            ostream << matrix.coeff(r, c);
            ostream << std::endl;
        }
    }

    return ostream;
}