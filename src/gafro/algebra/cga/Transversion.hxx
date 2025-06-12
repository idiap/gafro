// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Transversion.hpp>

namespace gafro
{

    template <typename T>
    Transversion<T>::Transversion() : Base(Parameters({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }))
    {}

    template <typename T>
    Transversion<T>::Transversion(const Generator &generator)
      : Base(Scalar<T>(TypeTraits<T>::One()) + Scalar<T>(TypeTraits<T>::Value(-0.5)) * generator)  //
    {}

    template <typename T>
    Transversion<T>::~Transversion()
    {}

    template <typename T>
    typename Transversion<T>::Generator Transversion<T>::log() const
    {
        return Generator(T(-2.0) * this->vector().bottomRows(3));
    }

    template <typename T>
    Transversion<T> Transversion<T>::exp(const T &e01, const T &e02, const T &e03)
    {
        return Transversion<T>::exp({ e01, e02, e03 });
    }

    template <typename T>
    Transversion<T> Transversion<T>::exp(const Eigen::Vector<T, 3> &generator)
    {
        return Transversion<T>::exp(Generator(generator));
    }

    template <typename T>
    Transversion<T> Transversion<T>::exp(const Generator &generator)
    {
        return Scalar<T>(TypeTraits<T>::One()) + Scalar<T>(TypeTraits<T>::Value(-0.5)) * generator;
    }

}  // namespace gafro