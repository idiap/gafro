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