// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Translator.hpp>
#include <gafro/algebra/cga/TranslatorGenerator.hpp>

namespace gafro
{

    template <typename T>
    Translator<T>::Translator()
      : Base(Parameters({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }))
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
        return this->log().vector();
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
    Translator<T> Translator<T>::exp(const T &e1i, const T &e2i, const T &e3i)
    {
        return Translator<T>::exp({ e1i, e2i, e3i });
    }

    template <typename T>
    Translator<T> Translator<T>::exp(const Eigen::Vector<T, 3> &generator)
    {
        return Translator<T>::exp(Generator(generator));
    }

    template <typename T>
    Translator<T> Translator<T>::exp(const Generator &generator)
    {
        return Scalar<T>(TypeTraits<T>::One()) + Scalar<T>(TypeTraits<T>::Value(-0.5)) * generator;
    }

    template <typename T>
    Translator<T> Translator<T>::X(const T &e1i)
    {
        return Translator<T>::exp(1.0, 0.0, 0.0);
    }

    template <typename T>
    Translator<T> Translator<T>::Y(const T &e2i)
    {
        return Translator<T>::exp(0.0, 1.0, 0.0);
    }

    template <typename T>
    Translator<T> Translator<T>::Z(const T &e3i)
    {
        return Translator<T>::exp(0.0, 0.0, 1.0);
    }

}  // namespace gafro