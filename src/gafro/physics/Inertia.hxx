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

#include <gafro/physics/Inertia.hpp>

namespace gafro
{

    template <class T>
    Inertia<T>::Inertia(const Tensor &tensor) : tensor_(tensor)
    {}

    template <class T>
    typename Rotor<T>::Generator Inertia<T>::operator()(const typename Rotor<T>::Generator &bivector) const
    {
        const T &i00 = tensor_.coeff(0, 0);
        const T &i10 = tensor_.coeff(1, 0);
        const T &i20 = tensor_.coeff(2, 0);
        const T &i01 = tensor_.coeff(0, 1);
        const T &i11 = tensor_.coeff(1, 1);
        const T &i21 = tensor_.coeff(2, 1);
        const T &i02 = tensor_.coeff(0, 2);
        const T &i12 = tensor_.coeff(1, 2);
        const T &i22 = tensor_.coeff(2, 2);

        const T &w0 = bivector.e23();
        const T &w1 = bivector.e13();
        const T &w2 = bivector.e12();

        typename Rotor<T>::Generator generator;

        generator.template set<blades::e23>(-1.0 * (i00 * w0 - i01 * w1 + i02 * w2));
        generator.template set<blades::e13>((i10 * w0 - i11 * w1 + i12 * w2));
        generator.template set<blades::e12>(-1.0 * (i20 * w0 - i21 * w1 + i22 * w2));

        return generator;
    }

    template <class T>
    Multivector<T, blades::e1, blades::e2, blades::e3> Inertia<T>::operator()(const Multivector<T, blades::e1, blades::e2, blades::e3> &vector) const
    {
        return Multivector<T, blades::e1, blades::e2, blades::e3>(tensor_ * vector.vector());
    }

    template <class T>
    typename Inertia<T>::Tensor Inertia<T>::getRotatedTensor(const Tensor &rotation_matrix) const
    {
        return rotation_matrix * tensor_ * rotation_matrix.transpose();
    }

    template <class T>
    const typename Inertia<T>::Tensor &Inertia<T>::getTensor() const
    {
        return tensor_;
    }

    template <class T>
    Inertia<T> Inertia<T>::Zero()
    {
        return Inertia(Inertia::Tensor::Zero());
    }

}  // namespace gafro