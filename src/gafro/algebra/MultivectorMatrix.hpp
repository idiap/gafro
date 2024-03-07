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

namespace gafro
{
    template <class T, template <class S> class M, int rows, int cols>
    class MultivectorMatrix
    {
      public:
        using Type = M<T>;
        using Matrix = Eigen::Matrix<T, rows * Type::size, cols>;

        MultivectorMatrix();

        virtual ~MultivectorMatrix();

        Type &getCoefficient(int row, int col);

        const Type &getCoefficient(int row, int col) const;

        void setCoefficient(int row, int col, const Type &multivector);

        Matrix embed() const;

        MultivectorMatrix reverse() const;

      protected:
      private:
        std::array<std::array<Type, cols>, rows> multivectors_;

      public:
        MultivectorMatrix &operator+=(const Type &multivector);

        MultivectorMatrix &operator*=(const Type &multivector);

        MultivectorMatrix operator*(const Type &multivector) const;
    };

    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols>::MultivectorMatrix()
    {}

    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols>::~MultivectorMatrix()
    {}

    template <class T, template <class S> class M, int rows, int cols>
    typename MultivectorMatrix<T, M, rows, cols>::Type &MultivectorMatrix<T, M, rows, cols>::getCoefficient(int row, int col)
    {
        return multivectors_[row][col];
    }

    template <class T, template <class S> class M, int rows, int cols>
    const typename MultivectorMatrix<T, M, rows, cols>::Type &MultivectorMatrix<T, M, rows, cols>::getCoefficient(int row, int col) const
    {
        return multivectors_[row][col];
    }

    template <class T, template <class S> class M, int rows, int cols>
    void MultivectorMatrix<T, M, rows, cols>::setCoefficient(int row, int col, const Type &multivector)
    {
        multivectors_[row][col] = multivector;
    }

    template <class T, template <class S> class M, int rows, int cols>
    Eigen::Matrix<T, rows * M<T>::size, cols> MultivectorMatrix<T, M, rows, cols>::embed() const
    {
        Matrix embedding = Matrix::Zero();

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                embedding.block(r * Type::size, c, Type::size, 1) = multivectors_[r][c].vector();
            }
        }

        return embedding;
    }

    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols> MultivectorMatrix<T, M, rows, cols>::reverse() const
    {
        MultivectorMatrix reversed;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                reversed[r][c] = multivectors_[r][c];
            }
        }

        return reversed;
    }

    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols> &MultivectorMatrix<T, M, rows, cols>::operator*=(const Type &multivector)
    {
        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                multivectors_[r][c] *= multivector;
            }
        }
    }

    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols> MultivectorMatrix<T, M, rows, cols>::operator*(const Type &multivector) const
    {
        MultivectorMatrix<T, M, rows, cols> result = *this;

        result *= multivector;

        return result;
    }

}  // namespace gafro

template <class T, template <class S> class M, int rows, int cols>
std::ostream &operator<<(std::ostream &ostream, const gafro::MultivectorMatrix<T, M, rows, cols> &matrix)
{
    ostream << std::endl;
    for (unsigned r = 0; r < rows; r++)
    {
        for (unsigned c = 0; c < cols; c++)
        {
            std::cout << r << "," << c << ":\t";
            ostream << matrix.getCoefficient(r, c);
            ostream << std::endl;
        }
    }

    return ostream;
}