// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

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

        this->setCoefficient(0, 0, InertiaElement<T>({ mass, zero, zero, zero, zero, zero }));
        this->setCoefficient(0, 1, InertiaElement<T>({ zero, mass, zero, zero, zero, zero }));
        this->setCoefficient(0, 2, InertiaElement<T>({ zero, zero, izz, zero, -iyz, ixz }));
        this->setCoefficient(0, 3, InertiaElement<T>({ zero, zero, zero, mass, zero, zero }));
        this->setCoefficient(0, 4, InertiaElement<T>({ zero, zero, -iyz, zero, iyy, -ixy }));
        this->setCoefficient(0, 5, InertiaElement<T>({ zero, zero, ixz, zero, -ixy, ixx }));
    }

    template <class T>
    Inertia<T>::Inertia(const T &mass, const Eigen::Matrix<T, 3, 3> &tensor)
      : Inertia<T>(mass, tensor.coeff(0, 0), tensor.coeff(0, 1), tensor.coeff(0, 2), tensor.coeff(1, 1), tensor.coeff(1, 2), tensor.coeff(2, 2))
    {}

    template <class T>
    Inertia<T>::Inertia(const Eigen::Matrix<T, 3, 3> &t, const Eigen::Matrix<T, 3, 3> &r)
    {
        static T zero = TypeTraits<T>::Zero();

        this->getCoefficient(0, 0) = InertiaElement<T>({ t.coeff(0, 0), t.coeff(0, 1), zero, t.coeff(0, 2), zero, zero });    // 01
        this->getCoefficient(0, 1) = InertiaElement<T>({ t.coeff(1, 0), t.coeff(1, 1), zero, t.coeff(1, 2), zero, zero });    // 02
        this->getCoefficient(0, 2) = InertiaElement<T>({ zero, zero, r.coeff(2, 2), zero, -r.coeff(1, 2), r.coeff(0, 2) });   // 12
        this->getCoefficient(0, 3) = InertiaElement<T>({ t.coeff(2, 0), t.coeff(2, 1), zero, t.coeff(2, 2), zero, zero });    // 03
        this->getCoefficient(0, 4) = InertiaElement<T>({ zero, zero, -r.coeff(2, 1), zero, r.coeff(1, 1), -r.coeff(0, 1) });  // 13
        this->getCoefficient(0, 5) = InertiaElement<T>({ zero, zero, r.coeff(2, 0), zero, -r.coeff(1, 0), r.coeff(0, 0) });   // 23
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
    Inertia<T>::Inertia(const Base &base) : Base(base)
    {}

    template <class T>
    template <class S>
    Inertia<T>::Inertia(const Inertia<S> &other)
    {
        this->setCoefficient(0, 0, other.getElement01());
        this->setCoefficient(0, 1, other.getElement02());
        this->setCoefficient(0, 2, other.getElement12());
        this->setCoefficient(0, 3, other.getElement03());
        this->setCoefficient(0, 4, other.getElement13());
        this->setCoefficient(0, 5, other.getElement23());
    }

    template <class T>
    Inertia<T> &Inertia<T>::operator+=(const Inertia &inertia)
    {
        this->getCoefficient(0, 0) = this->getCoefficient(0, 0) + inertia.getElement01();
        this->getCoefficient(0, 1) = this->getCoefficient(0, 1) + inertia.getElement02();
        this->getCoefficient(0, 2) = this->getCoefficient(0, 2) + inertia.getElement12();
        this->getCoefficient(0, 3) = this->getCoefficient(0, 3) + inertia.getElement03();
        this->getCoefficient(0, 4) = this->getCoefficient(0, 4) + inertia.getElement13();
        this->getCoefficient(0, 5) = this->getCoefficient(0, 5) + inertia.getElement23();

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
        return Wrench<T>(-(getElement23() | twist).template get<blades::scalar>(),  //
                         -(getElement13() | twist).template get<blades::scalar>(),  //
                         -(getElement12() | twist).template get<blades::scalar>(),  //
                         -(getElement01() | twist).template get<blades::scalar>(),  //
                         -(getElement02() | twist).template get<blades::scalar>(),  //
                         -(getElement03() | twist).template get<blades::scalar>());
    }

    template <class T>
    Twist<T> Inertia<T>::operator()(const Wrench<T> &wrench) const
    {
        typename Base::Matrix inverse_matrix = getTensor().inverse();

        Wrench<T> transformed_wrench = typename Wrench<T>::Parameters(inverse_matrix * wrench.vector());

        Twist<T> twist;

        twist.template set<blades::e23>(transformed_wrench.template get<blades::e23>());
        twist.template set<blades::e13>(transformed_wrench.template get<blades::e13>());
        twist.template set<blades::e12>(transformed_wrench.template get<blades::e12>());
        twist.template set<blades::e1i>(transformed_wrench.template get<blades::e01>());
        twist.template set<blades::e2i>(transformed_wrench.template get<blades::e02>());
        twist.template set<blades::e3i>(transformed_wrench.template get<blades::e03>());

        return twist;
    }

    template <class T>
    Inertia<T> Inertia<T>::transform(const Motor<T> &motor) const
    {
        InertiaElement<T> e1 = motor.apply(getElement01());
        InertiaElement<T> e2 = motor.apply(getElement02());
        InertiaElement<T> e3 = motor.apply(getElement12());
        InertiaElement<T> e4 = motor.apply(getElement03());
        InertiaElement<T> e5 = motor.apply(getElement13());
        InertiaElement<T> e6 = motor.apply(getElement23());

        return Inertia({
          motor.apply(InertiaElement<T>({ e1.template get<blades::e01>(), e2.template get<blades::e01>(), e3.template get<blades::e01>(),
                                          e4.template get<blades::e01>(), e5.template get<blades::e01>(), e6.template get<blades::e01>() })),
          motor.apply(InertiaElement<T>({ e1.template get<blades::e02>(), e2.template get<blades::e02>(), e3.template get<blades::e02>(),
                                          e4.template get<blades::e02>(), e5.template get<blades::e02>(), e6.template get<blades::e02>() })),
          motor.apply(InertiaElement<T>({ e1.template get<blades::e12>(), e2.template get<blades::e12>(), e3.template get<blades::e12>(),
                                          e4.template get<blades::e12>(), e5.template get<blades::e12>(), e6.template get<blades::e12>() })),
          motor.apply(InertiaElement<T>({ e1.template get<blades::e03>(), e2.template get<blades::e03>(), e3.template get<blades::e03>(),
                                          e4.template get<blades::e03>(), e5.template get<blades::e03>(), e6.template get<blades::e03>() })),
          motor.apply(InertiaElement<T>({ e1.template get<blades::e13>(), e2.template get<blades::e13>(), e3.template get<blades::e13>(),
                                          e4.template get<blades::e13>(), e5.template get<blades::e13>(), e6.template get<blades::e13>() })),
          motor.apply(InertiaElement<T>({ e1.template get<blades::e23>(), e2.template get<blades::e23>(), e3.template get<blades::e23>(),
                                          e4.template get<blades::e23>(), e5.template get<blades::e23>(), e6.template get<blades::e23>() })),
        });
    }

    template <class T>
    Inertia<T> Inertia<T>::inverseTransform(const Motor<T> &motor) const
    {
        return transform(motor.reverse());
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement01() const
    {
        return this->getCoefficient(0, 0);
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement02() const
    {
        return this->getCoefficient(0, 1);
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement12() const
    {
        return this->getCoefficient(0, 2);
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement03() const
    {
        return this->getCoefficient(0, 3);
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement13() const
    {
        return this->getCoefficient(0, 4);
    }

    template <class T>
    const InertiaElement<T> &Inertia<T>::getElement23() const
    {
        return this->getCoefficient(0, 5);
    }

    template <class T>
    typename Inertia<T>::Base::Matrix Inertia<T>::getTensor() const
    {
        return this->embed();
    }

    template <class T>
    const typename Inertia<T>::Base &Inertia<T>::getMultivectorMatrix() const
    {
        return *this;
    }

    template <class T>
    Inertia<T> Inertia<T>::Zero()
    {
        return Inertia(TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(),
                       TypeTraits<T>::Zero(), TypeTraits<T>::Zero());
    }

    template <class T>
    Inertia<T> Inertia<T>::Random()
    {
        Eigen::Vector<T, 7> values = Eigen::Vector<T, 7>::Random().array().abs();

        return Inertia<T>(values[0], values[1], values[2], values[3], values[4], values[5], values[6]);
    }

    template <class T>
    Inertia<T> Inertia<T>::RandomDiagonal()
    {
        Eigen::Vector<T, 4> values = Eigen::Vector<T, 4>::Random().array().abs();

        return Inertia<T>(values[0], values[1], 0.0, 0.0, values[2], 0.0, values[3]);
    }

    template <class T>
    std::ostream &operator<<(std::ostream &ostream, const Inertia<T> &inertia)
    {
        ostream << inertia.getElement23() << std::endl;
        ostream << inertia.getElement13() << std::endl;
        ostream << inertia.getElement12() << std::endl;
        ostream << inertia.getElement01() << std::endl;
        ostream << inertia.getElement02() << std::endl;
        ostream << inertia.getElement03() << std::endl;

        return ostream;
    }

}  // namespace gafro