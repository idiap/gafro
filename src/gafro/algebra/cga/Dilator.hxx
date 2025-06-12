// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Dilator.hpp>

namespace gafro
{

    template <class T>
    Dilator<T>::Dilator() = default;

    template <class T>
    Dilator<T>::Dilator(const T &dilation)
      : Base({ cosh(TypeTraits<T>::Value(0.5) * log(dilation)),  //
               sinh(TypeTraits<T>::Value(0.5) * log(dilation)) })
    {}

    template <class T>
    Dilator<T>::~Dilator() = default;

    template <class T>
    typename Dilator<T>::Generator Dilator<T>::logarithm() const
    {
        return Generator(TypeTraits<T>::Value(2.0) * acosh(this->template get<blades::scalar>()));
    }

    template <class T>
    Dilator<T> Dilator<T>::exp(const Generator &generator)
    {
        return Scalar<T>(cosh(TypeTraits<T>::Value(0.5) * generator.norm())) -
               sinh(TypeTraits<T>::Value(0.5) * generator.norm()) * generator / generator.signedNorm();
    }

}  // namespace gafro