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
//
// #include <gafro/algebra/RotorGenerator.hpp>

namespace gafro
{

    template <class T>
    Rotor<T>::Rotor() : Base(Parameters({ 1.0, 0.0, 0.0, 0.0 }))
    {}

    template <typename T>
    Rotor<T>::Rotor(const Base &other) : Base(other)
    {}

    template <typename T>
    Rotor<T>::Rotor(const Rotor<T>::Generator &bivector, const T &angle)
      : Rotor<T>::Base((Scalar<T>(cos(0.5 * angle)) + Scalar<T>(-sin(0.5 * angle)) * bivector).evaluate())
    {}

    template <typename T>
    Rotor<T>::Rotor(const Parameters &parameters) : Base(parameters)
    {}

    template <typename T>
    template <class E>
    Rotor<T>::Rotor(const Expression<E, Base> &expression)
    {
        this->template set<blades::scalar>(expression.template get<blades::scalar>());
        this->template set<blades::e23>(expression.template get<blades::e23>());
        this->template set<blades::e13>(expression.template get<blades::e13>());
        this->template set<blades::e12>(expression.template get<blades::e12>());
    }

    template <typename T>
    Rotor<T>::~Rotor()
    {}

    template <typename T>
    T Rotor<T>::angle() const
    {
        return T(2.0) * acos(scalar());
    }

    template <typename T>
    typename Rotor<T>::Generator Rotor<T>::log() const
    {
        T half_angle = acos(scalar());

        if (abs(half_angle) < 1e-6)
        {
            return Rotor<T>::Generator({ e23(), e13(), e12() });
        }

        return Rotor<T>::Generator({ e23() * T(-1.0 / (sin(half_angle))), e13() * T(-1.0 / (sin(half_angle))), e12() * T(-1.0 / (sin(half_angle))) },
                                   false);
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
        static std::array<Multivector<T>, 3> e_i = { E1<T>(), E2<T>(), E3<T>() };

        Eigen::Matrix<T, 3, 3> rotation_matrix;

        for (unsigned r = 0; r < 3; ++r)
        {
            const Multivector<T> &e_r = e_i[r];

            for (unsigned c = 0; c < 3; ++c)
            {
                const Multivector<T> &e_c = e_i[c];

                rotation_matrix.coeffRef(r, c) = (this->apply(e_c).mv() | e_r).scalar();
            }
        }

        return rotation_matrix;
    }

    template <class T>
    template <class Object>
    SandwichProduct<Object, Rotor<T>> Rotor<T>::apply(const Object &object)
    {
        return SandwichProduct(*static_cast<const Object *>(&object), *static_cast<const Rotor *>(this));
    }

}  // namespace gafro