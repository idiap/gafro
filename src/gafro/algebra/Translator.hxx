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

#include <gafro/algebra/Translator.hpp>
#include <gafro/algebra/TranslatorGenerator.hpp>

namespace gafro
{

    template <typename T>
    Translator<T>::Translator() : Base(Parameters({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }))
    {}

    template <typename T>
    Translator<T>::Translator(const Generator &generator)
      : Base(Scalar<T>(TypeTraits<T>::One()) + Scalar<T>(TypeTraits<T>::Value(-0.5)) * generator)  //
    {}

    template <typename T>
    Translator<T>::~Translator()
    {}

    template <typename T>
    typename Translator<T>::Generator Translator<T>::log() const
    {
        return Generator(T(-2.0) * this->vector().bottomRows(3));
    }

    template <typename T>
    Eigen::Vector<T, 3> Translator<T>::toTranslationVector() const
    {
        return T(-2.0) * this->vector().bottomRows(3);
    }

    template <typename T>
    Eigen::Matrix<T, 3, 3> Translator<T>::toSkewSymmetricMatrix() const
    {
        auto t = toTranslationVector();

        Eigen::Matrix<T, 3, 3> skew_symmetric_matrix;
        skew_symmetric_matrix << 0., -t[2], t[1],  //
          t[2], 0., -t[0],                         //
          -t[1], t[0], 0.;

        return skew_symmetric_matrix;
    }

    template <typename T>
    Translator<T> Translator<T>::exp(const Generator &generator)
    {
        return Scalar<T>(TypeTraits<T>::One()) + Scalar<T>(TypeTraits<T>::Value(-0.5)) * generator;
    }

}  // namespace gafro