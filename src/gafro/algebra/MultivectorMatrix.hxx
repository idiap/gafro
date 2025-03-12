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

#include <gafro/algebra/InnerProduct.hpp>
#include <gafro/algebra/MultivectorMatrix.hpp>
#include <gafro/algebra/OuterProduct.hpp>
#include <gafro/algebra/Sum.hpp>

namespace gafro
{
    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols>::MultivectorMatrix()
    {}

    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols>::MultivectorMatrix(const Matrix &parameters)
    {
        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                this->setCoefficient(r, c, Type(parameters.block(r * Type::size, c, Type::size, 1)));
            }
        }
    }

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
                reversed.setCoefficient(r, c, getCoefficient(r, c).reverse());
            }
        }

        return reversed;
    }

    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols> MultivectorMatrix<T, M, rows, cols>::inverse() const
    {
        MultivectorMatrix inversed;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                inversed.setCoefficient(r, c, getCoefficient(r, c).inverse());
            }
        }

        return inversed;
    }

    template <class T, template <class S> class M, int rows, int cols>
    auto MultivectorMatrix<T, M, rows, cols>::dual() const
    {
        typename Dual<M<T>>::Type::template Matrix<rows, cols> dualed;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                dualed.setCoefficient(r, c, getCoefficient(r, c).dual());
            }
        }

        return dualed;
    }

    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols> MultivectorMatrix<T, M, rows, cols>::normalized() const
    {
        MultivectorMatrix normalized;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                normalized.setCoefficient(r, c, getCoefficient(r, c).normalized());
            }
        }

        return normalized;
    }

    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols> MultivectorMatrix<T, M, rows, cols>::transform(const Motor<T> &motor) const
    {
        MultivectorMatrix transformed;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                transformed.setCoefficient(r, c, motor.apply(getCoefficient(r, c)));
            }
        }

        return transformed;
    }

    template <class T, template <class S> class M, int rows, int cols>
    template <template <class S2> class M2>
    MultivectorMatrix<T, M2, rows, cols> MultivectorMatrix<T, M, rows, cols>::extract() const
    {
        MultivectorMatrix<T, M2, rows, cols> matrix;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                matrix.setCoefficient(r, c, getCoefficient(r, c).template cast<M2<T>>());
            }
        }

        return matrix;
    }

    template <class T, template <class S> class M, int rows, int cols>
    std::vector<M<T>> MultivectorMatrix<T, M, rows, cols>::asVector() const
    {
        std::vector<M<T>> vector;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                vector.push_back(getCoefficient(r, c));
            }
        }

        return vector;
    }

    template <class T, template <class S> class M, int rows, int cols>
    MultivectorMatrix<T, M, rows, cols> &MultivectorMatrix<T, M, rows, cols>::operator*=(const Type &multivector)
    {
        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                setCoefficient(r, c, getCoefficient(r, c) * multivector);
            }
        }

        return *this;
    }

    template <class T, template <class S> class M, int rows, int cols>
    template <int... blades>
    auto MultivectorMatrix<T, M, rows, cols>::operator*(const typename Type::MAlgebra::template Multivector<T, blades...> &multivector) const
    {
        typename GeometricProduct<M<T>, typename Type::MAlgebra::template Multivector<T, blades...>>::Type::template Matrix<rows, cols> matrix;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                matrix.setCoefficient(r, c, getCoefficient(r, c) * multivector);
            }
        }

        return matrix;
    }

    template <class T, template <class S> class M, int rows, int cols>
    template <int... blades>
    auto MultivectorMatrix<T, M, rows, cols>::operator^(const typename Type::MAlgebra::template Multivector<T, blades...> &multivector) const
    {
        typename OuterProduct<M<T>, typename Type::MAlgebra::template Multivector<T, blades...>>::Type::template Matrix<rows, cols> matrix;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                matrix.setCoefficient(r, c, getCoefficient(r, c) ^ multivector);
            }
        }

        return matrix;
    }

    template <class T, template <class S> class M, int rows, int cols>
    template <int... blades>
    auto MultivectorMatrix<T, M, rows, cols>::operator|(const typename Type::MAlgebra::template Multivector<T, blades...> &multivector) const
    {
        typename InnerProduct<M<T>, typename Type::MAlgebra::template Multivector<T, blades...>>::Type::template Matrix<rows, cols> matrix;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                matrix.setCoefficient(r, c, getCoefficient(r, c) | multivector);
            }
        }

        return matrix;
    }

    template <class T, template <class S> class M, int rows, int cols>
    template <template <class S2> class M2, int rows2, int cols2>
    auto MultivectorMatrix<T, M, rows, cols>::operator*(const MultivectorMatrix<T, M2, rows2, cols2> &matrix) const
    {
        assert(cols == rows2);

        typename GeometricProduct<M<T>, M2<T>>::Type::template Matrix<rows, cols2> result;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols2; c++)
            {
                for (unsigned k = 0; k < cols; k++)
                {
                    result.setCoefficient(r, c, result.getCoefficient(r, c) + getCoefficient(r, k) * matrix.getCoefficient(k, c));
                }
            }
        }

        return result;
    }

    template <class T, template <class S> class M, int rows, int cols>
    template <template <class S2> class M2, int rows2, int cols2>
    auto MultivectorMatrix<T, M, rows, cols>::operator|(const MultivectorMatrix<T, M2, rows2, cols2> &matrix) const
    {
        assert(cols == rows2);

        typename InnerProduct<M<T>, M2<T>>::Type::template Matrix<rows, cols2> result;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols2; c++)
            {
                for (unsigned k = 0; k < cols; k++)
                {
                    result.setCoefficient(r, c, result.getCoefficient(r, c) + getCoefficient(r, k) | matrix.getCoefficient(k, c));
                }
            }
        }

        return result;
    }

    template <class T, template <class S> class M, int rows, int cols>
    template <template <class S2> class M2>
    auto MultivectorMatrix<T, M, rows, cols>::operator+(const MultivectorMatrix<T, M2, rows, cols> &matrix) const
    {
        typename Sum<M<T>, M2<T>, detail::AdditionOperator>::Type::template Matrix<rows, cols> result;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                result.setCoefficient(r, c, getCoefficient(r, c) + matrix.getCoefficient(r, c));
            }
        }

        return result;
    }

    template <class T, template <class S> class M, int rows, int cols>
    template <template <class S2> class M2>
    auto MultivectorMatrix<T, M, rows, cols>::operator-(const MultivectorMatrix<T, M2, rows, cols> &matrix) const
    {
        typename Sum<M<T>, M2<T>, detail::SubstractionOperator>::Type::template Matrix<rows, cols> result;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                result.setCoefficient(r, c, getCoefficient(r, c) - matrix.getCoefficient(r, c));
            }
        }

        return result;
    }

    // template <class T, template <class S> class M, int rows, int cols>
    // MultivectorMatrix<T, M, rows, cols> MultivectorMatrix<T, M, rows, cols>::operator|(const MultivectorMatrix &matrix) const
    // {
    //     MultivectorMatrix<T, M, rows, cols> result;

    //     for (unsigned r = 0; r < rows; r++)
    //     {
    //         for (unsigned c = 0; c < cols; c++)
    //         {
    //             result.setCoefficient(r, c, getCoefficient(r, c) | matrix.getCoefficient(r, c));
    //         }
    //     }

    //     return result;
    // }

    // template <class T, template <class S> class M, int rows, int cols>
    // MultivectorMatrix<T, M, rows, cols> MultivectorMatrix<T, M, rows, cols>::operator+(const MultivectorMatrix &matrix) const
    // {
    //     MultivectorMatrix<T, M, rows, cols> result;

    //     for (unsigned r = 0; r < rows; r++)
    //     {
    //         for (unsigned c = 0; c < cols; c++)
    //         {
    //             result.setCoefficient(r, c, getCoefficient(r, c) + matrix.getCoefficient(r, c));
    //         }
    //     }

    //     return result;
    // }

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

template <class T, template <class S> class M, int rows, int cols>
gafro::MultivectorMatrix<T, M, rows, cols> operator*(const T &value, const gafro::MultivectorMatrix<T, M, rows, cols> &matrix)
{
    gafro::MultivectorMatrix<T, M, rows, cols> result;

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            result.setCoefficient(r, c, value * matrix.getCoefficient(r, c));
        }
    }

    return result;
}

// template <class T, template <class S> class M, int rows, int cols>
// gafro::MultivectorMatrix<T, M, rows, cols> operator*(const typename gafro::MultivectorMatrix<T, M, rows, cols>::Type &multivector,
//                                                      const gafro::MultivectorMatrix<T, M, rows, cols> &matrix)
// {
//     gafro::MultivectorMatrix<T, M, rows, cols> result;

//     for (int r = 0; r < rows; ++r)
//     {
//         for (int c = 0; c < cols; ++c)
//         {
//             result.setCoefficient(r, c, multivector * matrix.getCoefficient(r, c));
//         }
//     }

//     return result;
// }