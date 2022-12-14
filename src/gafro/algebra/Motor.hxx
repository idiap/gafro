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

#include <gafro/algebra/Motor.hpp>
#include <gafro/algebra/expressions/MotorExponential.hpp>
#include <gafro/algebra/expressions/MotorLogarithm.hpp>

namespace gafro
{

    template <class T>
    Motor<T>::Motor() : Base(Unit())
    {}

    template <class T>
    Motor<T>::Motor(const Motor<T> &other) : Base(other.vector())
    {}

    template <class T>
    Motor<T>::Motor(const Base &other) : Base(other)
    {}

    template <class T>
    Motor<T>::Motor(Motor &&other) : Base(std::move(other))
    {}

    template <class T>
    Motor<T>::Motor(const Generator &generator)
    {
        // if (generator[blades::e23] * generator[blades::e23]      //
        //       + generator[blades::e13] * generator[blades::e13]  //
        //       + generator[blades::e12] * generator[blades::e12] >
        //     T(0.0))
        // {
        //     Multivector<T> b = dual_line[0] * blades::E23<T>() + dual_line[1] * blades::E13<T>() + dual_line[2] * blades::E12<T>();

        //     T theta = b.norm();

        //     Multivector<T> t = dual_line[3] * blades::E1<T>() + dual_line[4] * blades::E2<T>() + dual_line[5] * blades::E3<T>();

        //     b.normalize();

        //     *this = Motor<T>::exp(DualLine<T>(theta, b, t));
        // }
        // else
        // {
        //     Parameters parameters = Parameters::Zero();

        //     parameters.middleRows(1, 6) = generator.vector();

        //     this->setParameters(parameters);
        // }
    }

    template <class T>
    Motor<T>::Motor(const Translator<T> &translator, const Rotor<T> &rotor) : Motor<T>(translator * rotor)
    {}

    template <class T>
    Motor<T>::Motor(const Rotor<T> &rotor, const Translator<T> &translator) : Motor<T>(rotor * translator)
    {}

    template <class T>
    Motor<T>::Motor(const Rotor<T> &rotor) : Motor(Translator<T>(), rotor)
    {}

    template <class T>
    Motor<T>::Motor(const Parameters &parameters) : Base(parameters)
    {}

    template <class T>
    template <class E>
    Motor<T>::Motor(const Expression<E, Motor> &expression)
    {
        this->template set<blades::scalar>(expression.template get<blades::scalar>());
        this->template set<blades::e23>(expression.template get<blades::e23>());
        this->template set<blades::e13>(expression.template get<blades::e13>());
        this->template set<blades::e12>(expression.template get<blades::e12>());
        this->template set<blades::e1i>(expression.template get<blades::e1i>());
        this->template set<blades::e2i>(expression.template get<blades::e2i>());
        this->template set<blades::e3i>(expression.template get<blades::e3i>());
        this->template set<blades::e123i>(expression.template get<blades::e123i>());
    }

    template <class T>
    Motor<T> &Motor<T>::operator=(const Motor &other)
    {
        this->setParameters(other.vector());

        return *this;
    }

    template <class T>
    Motor<T> &Motor<T>::operator=(Motor<T> &&other)
    {
        this->setParameters(std::forward<Parameters>(other.vector()));

        return *this;
    }

    template <class T>
    template <class Object>
    SandwichProduct<Object, Motor<T>> Motor<T>::apply(const Object &object)
    {
        return SandwichProduct(*static_cast<const Object *>(&object), *static_cast<const Motor *>(this));
    }

    template <class T>
    Motor<T>::Motor(Base &&other) : Base(std::forward<Base>(other))
    {
        this->setParameters(std::move(other.vector()));
    }

    template <class T>
    Motor<T> &Motor<T>::operator=(Base &&other)
    {
        this->setParameters(std::move(other.vector()));

        return *this;
    }

    template <class T>
    Rotor<T> Motor<T>::getRotor() const
    {
        return (E0<T>(-1.0) | (*this) * Ei<T>(1.0));
    }

    template <class T>
    typename Motor<T>::Logarithm Motor<T>::log() const
    {
        return Logarithm(*static_cast<const Motor<T> *>(this));
    }

    template <class T>
    Eigen::Matrix<T, 6, 8> Motor<T>::logJacobian() const
    {
        const T &m1 = this->vector().coeff(0, 0);
        const T &m2 = this->vector().coeff(1, 0);
        const T &m3 = this->vector().coeff(2, 0);
        const T &m4 = this->vector().coeff(3, 0);
        const T &m5 = this->vector().coeff(4, 0);
        const T &m6 = this->vector().coeff(5, 0);
        const T &m7 = this->vector().coeff(6, 0);
        const T &m8 = this->vector().coeff(7, 0);

        Eigen::Matrix<T, 6, 8> jacobian = Eigen::Matrix<T, 6, 8>::Zero();

        T factor1 = 0.0;
        T factor2 = 0.0;

        if (m1 != 1.0)
        {
            factor1 = -2.0 * (1.0 / (m1 * m1 - 1.0) + m1 * acos(m1) / pow(1.0 - m1 * m1, 1.5));
            factor2 = -(2.0 * acos(m1) / sin(acos(m1)));
        }

        jacobian.coeffRef(0, 0) = factor1 * m2;
        jacobian.coeffRef(0, 1) = factor2;

        jacobian.coeffRef(1, 0) = factor1 * m3;
        jacobian.coeffRef(1, 2) = factor2;

        jacobian.coeffRef(2, 0) = factor1 * m4;
        jacobian.coeffRef(2, 3) = factor2;

        jacobian.coeffRef(3, 0) = -2.0 * m5;
        jacobian.coeffRef(3, 1) = -2.0 * m8;
        jacobian.coeffRef(3, 2) = -2.0 * m7;
        jacobian.coeffRef(3, 3) = -2.0 * m6;
        jacobian.coeffRef(3, 4) = -2.0 * m1;
        jacobian.coeffRef(3, 5) = -2.0 * m4;
        jacobian.coeffRef(3, 6) = -2.0 * m3;
        jacobian.coeffRef(3, 7) = -2.0 * m2;

        jacobian.coeffRef(4, 0) = -2.0 * m6;
        jacobian.coeffRef(4, 1) = -2.0 * m7;
        jacobian.coeffRef(4, 2) = 2.0 * m8;
        jacobian.coeffRef(4, 3) = 2.0 * m5;
        jacobian.coeffRef(4, 4) = 2.0 * m4;
        jacobian.coeffRef(4, 5) = -2.0 * m1;
        jacobian.coeffRef(4, 6) = -2.0 * m2;
        jacobian.coeffRef(4, 7) = 2.0 * m3;

        jacobian.coeffRef(5, 0) = -2.0 * m7;
        jacobian.coeffRef(5, 1) = 2.0 * m6;
        jacobian.coeffRef(5, 2) = 2.0 * m5;
        jacobian.coeffRef(5, 3) = -2.0 * m8;
        jacobian.coeffRef(5, 4) = 2.0 * m3;
        jacobian.coeffRef(5, 5) = 2.0 * m2;
        jacobian.coeffRef(5, 6) = -2.0 * m1;
        jacobian.coeffRef(5, 7) = -2.0 * m4;

        return jacobian;
    }

    template <class T>
    Motor<T> Motor<T>::Unit()
    {
        Parameters parameters({ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

        return Motor<T>(parameters);
    }

    template <class T>
    typename Motor<T>::Exponential Motor<T>::exp(const Generator &generator)
    {
        return Exponential(*static_cast<const Generator *>(&generator));
    }

}  // namespace gafro