// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/dqa/DualQuaternion.hpp>
//
#include <gafro/algebra/cga/Motor.hxx>

namespace gafro::dqa
{
    template <class T>
    DualQuaternion<T>::DualQuaternion() = default;

    template <class T>
    DualQuaternion<T>::DualQuaternion(const Base &dq) : Base(dq)
    {}

    template <class T>
    DualQuaternion<T>::DualQuaternion(Base &&dq) : Base(std::move(dq))
    {}

    template <class T>
    DualQuaternion<T>::DualQuaternion(const Eigen::Quaternion<T> &quaternion)
    {
        this->template set<idx::scalar>(quaternion.w());
        this->template set<idx::i>(quaternion.x());
        this->template set<idx::j>(-quaternion.y());
        this->template set<idx::k>(-quaternion.z());
    }

    template <class T>
    DualQuaternion<T>::DualQuaternion(const Motor<T> &motor)
    {
        this->template set<idx::scalar>(motor.template get<gafro::blades::scalar>());
        this->template set<idx::i>(-motor.template get<gafro::blades::e23>());
        this->template set<idx::j>(-motor.template get<gafro::blades::e13>());
        this->template set<idx::k>(motor.template get<gafro::blades::e12>());
        this->template set<idx::e>(-motor.template get<gafro::blades::e123i>());
        this->template set<idx::ei>(-motor.template get<gafro::blades::e1i>());
        this->template set<idx::ej>(-motor.template get<gafro::blades::e2i>());
        this->template set<idx::ek>(-motor.template get<gafro::blades::e3i>());
    }

    template <class T>
    DualQuaternion<T>::DualQuaternion(const Point<T> &point)
    {
        this->template set<idx::scalar>(point.template get<idx::scalar>());
        this->template set<idx::ei>(point.template get<idx::ei>());
        this->template set<idx::ej>(point.template get<idx::ej>());
        this->template set<idx::ek>(point.template get<idx::ek>());

        this->template set<idx::e>(0.0);
        this->template set<idx::i>(0.0);
        this->template set<idx::j>(0.0);
        this->template set<idx::k>(0.0);
    }

    template <class T>
    DualQuaternion<T>::DualQuaternion(const Plane<T> &plane)
    {
        this->template set<idx::scalar>(0.0);
        this->template set<idx::ei>(0.0);
        this->template set<idx::ej>(0.0);
        this->template set<idx::ek>(0.0);

        this->template set<idx::e>(plane.template get<idx::e>());
        this->template set<idx::i>(plane.template get<idx::i>());
        this->template set<idx::j>(plane.template get<idx::j>());
        this->template set<idx::k>(plane.template get<idx::k>());
    }

    template <class T>
    DualQuaternion<T>::~DualQuaternion() = default;

    template <class T>
    DualQuaternion<T>::operator Motor<T>() const
    {
        Motor<T> motor;

        motor.template set<gafro::blades::scalar>(this->template get<idx::scalar>());
        motor.template set<gafro::blades::e23>(-this->template get<idx::i>());
        motor.template set<gafro::blades::e13>(-this->template get<idx::j>());
        motor.template set<gafro::blades::e12>(this->template get<idx::k>());
        motor.template set<gafro::blades::e123i>(-this->template get<idx::e>());
        motor.template set<gafro::blades::e1i>(-this->template get<idx::ei>());
        motor.template set<gafro::blades::e2i>(-this->template get<idx::ej>());
        motor.template set<gafro::blades::e3i>(-this->template get<idx::ek>());

        return motor;
    }

    template <class T>
    DualQuaternion<T> DualQuaternion<T>::conjugate() const
    {
        DualQuaternion dq;

        dq.template set<idx::scalar>(this->template get<idx::scalar>());
        dq.template set<idx::i>(-this->template get<idx::i>());
        dq.template set<idx::j>(-this->template get<idx::j>());
        dq.template set<idx::k>(-this->template get<idx::k>());
        dq.template set<idx::e>(this->template get<idx::e>());
        dq.template set<idx::ei>(-this->template get<idx::ei>());
        dq.template set<idx::ej>(-this->template get<idx::ej>());
        dq.template set<idx::ek>(-this->template get<idx::ek>());

        return dq;
    }

    template <class T>
    DualQuaternion<T> DualQuaternion<T>::sharpConjugate() const
    {
        DualQuaternion dq;

        dq.template set<idx::scalar>(this->template get<idx::scalar>());
        dq.template set<idx::i>(-this->template get<idx::i>());
        dq.template set<idx::j>(-this->template get<idx::j>());
        dq.template set<idx::k>(-this->template get<idx::k>());
        dq.template set<idx::e>(-this->template get<idx::e>());
        dq.template set<idx::ei>(this->template get<idx::ei>());
        dq.template set<idx::ej>(this->template get<idx::ej>());
        dq.template set<idx::ek>(this->template get<idx::ek>());

        return dq;
    }

    template <class T>
    DualQuaternion<T> DualQuaternion<T>::sharp() const
    {
        DualQuaternion dq;

        dq.template set<idx::scalar>(this->template get<idx::scalar>());
        dq.template set<idx::i>(this->template get<idx::i>());
        dq.template set<idx::j>(this->template get<idx::j>());
        dq.template set<idx::k>(this->template get<idx::k>());
        dq.template set<idx::e>(-this->template get<idx::e>());
        dq.template set<idx::ei>(-this->template get<idx::ei>());
        dq.template set<idx::ej>(-this->template get<idx::ej>());
        dq.template set<idx::ek>(-this->template get<idx::ek>());

        return dq;
    }

    template <class T>
    DualQuaternion<T>::Primal DualQuaternion<T>::getPrimal() const
    {
        Primal primal;

        primal.template set<idx::scalar>(this->template get<idx::scalar>());
        primal.template set<idx::i>(this->template get<idx::i>());
        primal.template set<idx::j>(this->template get<idx::j>());
        primal.template set<idx::k>(this->template get<idx::k>());

        return primal;
    }

    template <class T>
    DualQuaternion<T>::Dual DualQuaternion<T>::getDual() const
    {
        Dual dual;

        dual.template set<idx::e>(this->template get<idx::e>());
        dual.template set<idx::ei>(this->template get<idx::ei>());
        dual.template set<idx::ej>(this->template get<idx::ej>());
        dual.template set<idx::ek>(this->template get<idx::ek>());

        return dual;
    }

    template <class T>
    DualQuaternion<T>::Imaginary DualQuaternion<T>::getImaginary() const
    {
        Imaginary imaginary;

        imaginary.template set<idx::i>(this->template get<idx::i>());
        imaginary.template set<idx::j>(this->template get<idx::j>());
        imaginary.template set<idx::k>(this->template get<idx::k>());
        imaginary.template set<idx::ek>(this->template get<idx::ek>());
        imaginary.template set<idx::ej>(this->template get<idx::ej>());
        imaginary.template set<idx::ei>(this->template get<idx::ei>());

        return imaginary;
    }

}  // namespace gafro::dqa