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

#include <gafro/algebra/cga/Rotor.hpp>
#include <gafro/algebra/cga/RotorExponential.hpp>
#include <gafro/algebra/cga/Versor.hxx>

namespace gafro
{

    template <class T>
    Rotor<T>::Rotor() : Base(Parameters({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }))
    {}

    template <typename T>
    Rotor<T>::Rotor(const Rotor<T>::Generator &bivector, const T &angle)
      : Rotor<T>::Base((Scalar<T>(cos(TypeTraits<T>::Value(0.5) * angle)) + Scalar<T>(-sin(TypeTraits<T>::Value(0.5) * angle)) * bivector).evaluate())
    {}

    template <typename T>
    Rotor<T>::Rotor(const Parameters &parameters) : Base(parameters)
    {}

    template <typename T>
    Rotor<T> Rotor<T>::fromQuaternion(const Eigen::Quaternion<T> &quaternion)
    {
        Rotor rotor(Parameters({ quaternion.w(), quaternion.z(), -quaternion.y(), quaternion.x() }));

        return rotor.reverse();
    }

    template <typename T>
    Rotor<T>::~Rotor()
    {}

    template <typename T>
    T Rotor<T>::angle() const
    {
        return TypeTraits<T>::Value(2.0) * acos(scalar());
    }

    template <typename T>
    typename Rotor<T>::Generator Rotor<T>::log() const
    {
        T angle = acos(scalar());

        if (abs(angle) < 1e-6)
        {
            return Rotor<T>::Generator({ e12(), e13(), e23() });
        }

        angle = -2.0 * angle / (sin(angle));

        return Rotor<T>::Generator({ angle * e12(),  //
                                     angle * e13(),  //
                                     angle * e23() });
    }

    template <typename T>
    typename Rotor<T>::Exponential Rotor<T>::exp(const T &e12, const T &e13, const T &e23)
    {
        return Rotor<T>::exp({ e12, e13, e23 });
    }

    template <typename T>
    typename Rotor<T>::Exponential Rotor<T>::exp(const Eigen::Vector<T, 3> &generator)
    {
        return Rotor<T>::exp(Generator(generator));
    }

    template <typename T>
    typename Rotor<T>::Exponential Rotor<T>::exp(const Generator &generator)
    {
        return Exponential(*static_cast<const Generator *>(&generator));
    }

    template <typename T>
    const T &Rotor<T>::scalar() const
    {
        return this->template get<blades::scalar>();
    }

    template <typename T>
    const T &Rotor<T>::e23() const
    {
        return this->template get<blades::e23>();
    }

    template <typename T>
    const T &Rotor<T>::e13() const
    {
        return this->template get<blades::e13>();
    }

    template <typename T>
    const T &Rotor<T>::e12() const
    {
        return this->template get<blades::e12>();
    }

    template <typename T>
    Eigen::Quaternion<T> Rotor<T>::quaternion() const
    {
        return Eigen::Quaternion<T>(this->reverse().template get<blades::scalar>(),  //
                                    this->reverse().template get<blades::e23>(),     //
                                    -this->reverse().template get<blades::e13>(),    //
                                    this->reverse().template get<blades::e12>());
    }

    template <typename T>
    Eigen::Matrix<T, 3, 3> Rotor<T>::toRotationMatrix() const
    {
        return Eigen::Matrix<T, 3, 3>(quaternion());
    }

}  // namespace gafro