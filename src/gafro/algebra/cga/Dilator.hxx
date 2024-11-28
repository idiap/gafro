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

#include <gafro/algebra/cga/Dilator.hpp>

namespace gafro
{

    template <class T>
    Dilator<T>::Dilator() = default;

    template <class T>
    Dilator<T>::Dilator(const T &dilation)
      : Base({ TypeTraits<T>::Value(cosh(0.5 * ::log(dilation))), TypeTraits<T>::Value(sinh(0.5 * ::log(dilation))) })
    {}

    template <class T>
    Dilator<T>::~Dilator() = default;

    template <class T>
    typename Dilator<T>::Generator Dilator<T>::log() const
    {
        return Generator(TypeTraits<T>::Value(2.0) * acosh(this->template get<blades::scalar>()));
    }

    template <class T>
    Dilator<T> Dilator<T>::exp(const Generator &generator)
    {
        return Scalar<T>(cosh(0.5 * generator.norm())) - sinh(0.5 * generator.norm()) * generator / generator.signedNorm();
    }

}  // namespace gafro