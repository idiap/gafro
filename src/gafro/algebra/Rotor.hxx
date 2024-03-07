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

#include <gafro/algebra/Rotor.hpp>
#include <gafro/algebra/Versor.hxx>
#include <gafro/algebra/expressions/RotorExponential.hpp>

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
        Rotor rotor(Parameters({ quaternion.w(), quaternion.x(), -quaternion.y(), quaternion.z() }));

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
        T acos = std::acos(scalar());

        if (abs(acos) < 1e-6)
        {
            return Rotor<T>::Generator({ e23(), e13(), e12() });
        }

        acos = -2.0 * acos / (sin(acos));

        return Rotor<T>::Generator({ e23() * TypeTraits<T>::Value(acos), e13() * TypeTraits<T>::Value(acos), e12() * TypeTraits<T>::Value(acos) });
    }

    template <typename T>
    typename Rotor<T>::Exponential Rotor<T>::exp(const Generator &generator)
    {
        return Exponential(*static_cast<const Generator *>(&generator));
    }

    template <typename T>
    const T &Rotor<T>::scalar() const
    {
        return this->vector().coeffRef(0, 0);
    }

    template <typename T>
    const T &Rotor<T>::e23() const
    {
        return this->vector().coeffRef(1, 0);
    }

    template <typename T>
    const T &Rotor<T>::e13() const
    {
        return this->vector().coeffRef(2, 0);
    }

    template <typename T>
    const T &Rotor<T>::e12() const
    {
        return this->vector().coeffRef(3, 0);
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