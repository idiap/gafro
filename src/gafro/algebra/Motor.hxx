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

    // CONSTRUCTORS

    template <class T>
    Motor<T>::Motor() : Base(Unit())
    {}

    template <class T>
    Motor<T>::Motor(const Motor<T> &other) : Base(other.vector())
    {}

    template <class T>
    Motor<T>::Motor(Motor &&other) : Base(std::move(other))
    {}

    template <class T>
    Motor<T>::Motor(const Generator &generator) : Base(Motor::exp(generator))
    {}

    template <class T>
    Motor<T>::Motor(const Translator<T> &translator) : Motor(translator, Rotor<T>())
    {}

    template <class T>
    Motor<T>::Motor(const Translator<T> &translator, const Rotor<T> &rotor) : Motor<T>(translator * rotor)
    {}

    template <class T>
    Motor<T>::Motor(const Rotor<T> &rotor, const Translator<T> &translator) : Motor<T>(rotor * translator)
    {}

    template <class T>
    Motor<T>::Motor(const Rotor<T> &rotor) : Motor(Translator<T>(), rotor)
    {}

    // OPERATORS

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
    Motor<T> &Motor<T>::operator*=(const Motor &other)
    {
        *this = Motor(Motor(*this) * other);

        return *this;
    }

    // MOTOR SPECIFIC FUNCTIONS

    template <class T>
    Rotor<T> Motor<T>::getRotor() const
    {
        return Rotor<T>(this->vector().topRows(4));
    }

    template <class T>
    Translator<T> Motor<T>::getTranslator() const
    {
        return (*this) * getRotor().reverse();
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
    Eigen::Matrix<T, 4, 4> Motor<T>::toTransformationMatrix() const
    {
        Eigen::Matrix<T, 4, 4> transformation_matrix = Eigen::Matrix<T, 4, 4>::Zero();

        transformation_matrix.block(0, 0, 3, 3) = getRotor().toRotationMatrix();
        transformation_matrix.block(0, 3, 3, 1) = getTranslator().toTranslationVector();
        transformation_matrix.coeffRef(3, 3) = 1.0;

        return transformation_matrix;
    }

    template <class T>
    Eigen::Matrix<T, 6, 6> Motor<T>::toAdjointMatrix() const
    {
        Eigen::Matrix<T, 6, 6> adjoint_matrix = Eigen::Matrix<T, 6, 6>::Zero();

        adjoint_matrix.block(0, 0, 3, 3) = getRotor().toRotationMatrix();
        adjoint_matrix.block(3, 3, 3, 3) = getRotor().toRotationMatrix();
        adjoint_matrix.block(3, 0, 3, 3) = getTranslator().toSkewSymmetricMatrix() * getRotor().toRotationMatrix();

        return adjoint_matrix;
    }

    template <class T>
    Eigen::Matrix<T, 6, 6> Motor<T>::toDualAdjointMatrix() const
    {
        Eigen::Matrix<T, 6, 6> adjoint_matrix = Eigen::Matrix<T, 6, 6>::Zero();

        adjoint_matrix.block(0, 0, 3, 3) = getRotor().toRotationMatrix().transpose();
        adjoint_matrix.block(3, 3, 3, 3) = getRotor().toRotationMatrix().transpose();
        adjoint_matrix.block(0, 3, 3, 3) = getRotor().toRotationMatrix().transpose() * getTranslator().toSkewSymmetricMatrix().transpose();

        return adjoint_matrix;
    }

    // STATIC FUNCTIONS

    template <class T>
    Motor<T> Motor<T>::Unit()
    {
        return Motor<T>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(),
                          TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() });
    }

    template <class T>
    Motor<T> Motor<T>::Random()
    {
        return Motor<T>::exp(Motor<T>::Generator::Random());
    }

    template <class T>
    typename Motor<T>::Exponential Motor<T>::exp(const Generator &generator)
    {
        return Exponential(*static_cast<const Generator *>(&generator));
    }

}  // namespace gafro