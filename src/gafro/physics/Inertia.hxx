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

#include <gafro/physics/Twist.hxx>
#include <gafro/physics/Wrench.hxx>
//
#include <gafro/physics/Inertia.hpp>

namespace gafro
{

    template <class T>
    Inertia<T>::Inertia() = default;

    template <class T>
    Inertia<T>::Inertia(const T &mass, const T &ixx, const T &ixy, const T &ixz, const T &iyy, const T &iyz, const T &izz)
    {
        static T zero = TypeTraits<T>::Zero();

        this->setCoefficient(0, 0, InertiaElement<T>({ ixx, -ixy, ixz, zero, zero, zero }));
        this->setCoefficient(0, 1, InertiaElement<T>({ -ixy, iyy, -iyz, zero, zero, zero }));
        this->setCoefficient(0, 2, InertiaElement<T>({ ixz, -iyz, izz, zero, zero, zero }));
        this->setCoefficient(0, 3, InertiaElement<T>({ zero, zero, zero, mass, zero, zero }));
        this->setCoefficient(0, 4, InertiaElement<T>({ zero, zero, zero, zero, mass, zero }));
        this->setCoefficient(0, 5, InertiaElement<T>({ zero, zero, zero, zero, zero, mass }));
    }

    template <class T>
    Inertia<T>::Inertia(const T &mass, const Eigen::Matrix<T, 3, 3> &tensor)
    {
        static T zero = TypeTraits<T>::Zero();

        this->setCoefficient(0, 0, InertiaElement<T>({ tensor.coeff(0, 0), -tensor.coeff(0, 1), tensor.coeff(0, 2), zero, zero, zero }));
        this->setCoefficient(0, 1, InertiaElement<T>({ -tensor.coeff(1, 0), tensor.coeff(1, 1), -tensor.coeff(1, 2), zero, zero, zero }));
        this->setCoefficient(0, 2, InertiaElement<T>({ tensor.coeff(2, 0), -tensor.coeff(2, 1), tensor.coeff(2, 2), zero, zero, zero }));
        this->setCoefficient(0, 3, InertiaElement<T>({ zero, zero, zero, mass, zero, zero }));
        this->setCoefficient(0, 4, InertiaElement<T>({ zero, zero, zero, zero, mass, zero }));
        this->setCoefficient(0, 5, InertiaElement<T>({ zero, zero, zero, zero, zero, mass }));
    }

    template <class T>
    Inertia<T>::Inertia(const std::array<InertiaElement<T>, 6> &elements)
    {
        this->setCoefficient(0, 0, elements[0]);
        this->setCoefficient(0, 1, elements[1]);
        this->setCoefficient(0, 2, elements[2]);
        this->setCoefficient(0, 3, elements[3]);
        this->setCoefficient(0, 4, elements[4]);
        this->setCoefficient(0, 5, elements[5]);
    }

    template <class T>
    template <class S>
    Inertia<T>::Inertia(const Inertia<S> &other)
    {
        this->setCoefficient(0, 0, other.getElement23());
        this->setCoefficient(0, 1, other.getElement13());
        this->setCoefficient(0, 2, other.getElement12());
        this->setCoefficient(0, 3, other.getElement01());
        this->setCoefficient(0, 4, other.getElement02());
        this->setCoefficient(0, 5, other.getElement03());
    }

    template <class T>
    Inertia<T> &Inertia<T>::operator+=(const Inertia &inertia)
    {
        this->getCoefficient(0, 0) = this->getCoefficient(0, 0) + inertia.getElement23();
        this->getCoefficient(0, 1) = this->getCoefficient(0, 1) + inertia.getElement13();
        this->getCoefficient(0, 2) = this->getCoefficient(0, 2) + inertia.getElement12();
        this->getCoefficient(0, 3) = this->getCoefficient(0, 3) + inertia.getElement01();
        this->getCoefficient(0, 4) = this->getCoefficient(0, 4) + inertia.getElement02();
        this->getCoefficient(0, 5) = this->getCoefficient(0, 5) + inertia.getElement03();

        return *this;
    }

    template <class T>
    Inertia<T> Inertia<T>::operator+(const Inertia<T> &inertia)
    {
        Inertia sum = *this;
        sum += inertia;

        return sum;
    }

    template <class T>
    Wrench<T> Inertia<T>::operator()(const Twist<T> &twist) const
    {
        return Wrench<T>({ -(getElement23() | twist).template get<blades::scalar>(),  //
                           -(getElement13() | twist).template get<blades::scalar>(),  //
                           -(getElement12() | twist).template get<blades::scalar>(),  //
                           -(getElement01() | twist).template get<blades::scalar>(),  //
                           -(getElement02() | twist).template get<blades::scalar>(),  //
                           -(getElement03() | twist).template get<blades::scalar>() });
    }

    template <class T>
    Inertia<T> Inertia<T>::transform(const Motor<T> &motor) const
    {
        InertiaElement<T> e1 = motor.apply(getElement23());
        InertiaElement<T> e2 = motor.apply(getElement13());
        InertiaElement<T> e3 = motor.apply(getElement12());
        InertiaElement<T> e4 = motor.apply(getElement01());
        InertiaElement<T> e5 = motor.apply(getElement02());
        InertiaElement<T> e6 = motor.apply(getElement03());

        return Inertia({
          motor.apply(InertiaElement<T>({ e1.template get<blades::e23>(), e2.template get<blades::e23>(), e3.template get<blades::e23>(),
                                          e4.template get<blades::e23>(), e5.template get<blades::e23>(), e6.template get<blades::e23>() })),
          motor.apply(InertiaElement<T>({ e1.template get<blades::e13>(), e2.template get<blades::e13>(), e3.template get<blades::e13>(),
                                          e4.template get<blades::e13>(), e5.template get<blades::e13>(), e6.template get<blades::e13>() })),
          motor.apply(InertiaElement<T>({ e1.template get<blades::e12>(), e2.template get<blades::e12>(), e3.template get<blades::e12>(),
                                          e4.template get<blades::e12>(), e5.template get<blades::e12>(), e6.template get<blades::e12>() })),
          motor.apply(InertiaElement<T>({ e1.template get<blades::e01>(), e2.template get<blades::e01>(), e3.template get<blades::e01>(),
                                          e4.template get<blades::e01>(), e5.template get<blades::e01>(), e6.template get<blades::e01>() })),
          motor.apply(InertiaElement<T>({ e1.template get<blades::e02>(), e2.template get<blades::e02>(), e3.template get<blades::e02>(),
                                          e4.template get<blades::e02>(), e5.template get<blades::e02>(), e6.template get<blades::e02>() })),
          motor.apply(InertiaElement<T>({ e1.template get<blades::e03>(), e2.template get<blades::e03>(), e3.template get<blades::e03>(),
                                          e4.template get<blades::e03>(), e5.template get<blades::e03>(), e6.template get<blades::e03>() })),
        });
    }

    template <class T>
    Inertia<T> Inertia<T>::inverseTransform(const Motor<T> &motor) const
    {
        return transform(motor.reverse());
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement23() const
    {
        return this->getCoefficient(0, 0);
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement13() const
    {
        return this->getCoefficient(0, 1);
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement12() const
    {
        return this->getCoefficient(0, 2);
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement01() const
    {
        return this->getCoefficient(0, 3);
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement02() const
    {
        return this->getCoefficient(0, 4);
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement03() const
    {
        return this->getCoefficient(0, 5);
    }

    template <class T>
    typename Inertia<T>::Base::Matrix Inertia<T>::getTensor() const
    {
        return this->embed();
    }

    template <class T>
    Inertia<T> Inertia<T>::Zero()
    {
        return Inertia(TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(),
                       TypeTraits<T>::Zero(), TypeTraits<T>::Zero());
    }

}  // namespace gafro