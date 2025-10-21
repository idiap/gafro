// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Motor.hpp>
#include <gafro/algebra/cga/MotorExponential.hpp>
#include <gafro/algebra/cga/MotorLogarithm.hpp>

namespace gafro
{

    // CONSTRUCTORS

    template <class T>
    Motor<T>::Motor()
      : Base(Unit())
    {}

    template <class T>
    Motor<T>::Motor(const Motor<T> &other)
      : Base(other.vector())
    {}

    template <class T>
    Motor<T>::Motor(Motor &&other)
      : Base(std::move(other))
    {}

    template <class T>
    Motor<T>::Motor(const Generator &generator)
      : Base(Motor::exp(generator))
    {}

    template <class T>
    Motor<T>::Motor(const Translator<T> &translator)
      : Motor(translator, Rotor<T>())
    {}

    template <class T>
    Motor<T>::Motor(const Translator<T> &translator, const Rotor<T> &rotor)
      : Motor<T>(translator * rotor)
    {}

    template <class T>
    Motor<T>::Motor(const Rotor<T> &rotor, const Translator<T> &translator)
      : Motor<T>(rotor * translator)
    {}

    template <class T>
    Motor<T>::Motor(const Rotor<T> &rotor)
      : Motor(Translator<T>(), rotor)
    {}

    template <class T>
    Motor<T>::Motor(const Eigen::Vector<T, 3> &position, const Eigen::Quaternion<T> &orientation)
      : Motor<T>(Translator<T>::exp(position), Rotor<T>::fromQuaternion(orientation))
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
        Rotor<T> rotor;

        rotor.template set<blades::scalar>(this->template get<blades::scalar>());
        rotor.template set<blades::e23>(this->template get<blades::e23>());
        rotor.template set<blades::e13>(this->template get<blades::e13>());
        rotor.template set<blades::e12>(this->template get<blades::e12>());

        return rotor;
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
    Eigen::Matrix<T, 4, 4> Motor<T>::toTransformationMatrix() const
    {
        Eigen::Matrix<T, 4, 4> transformation_matrix = Eigen::Matrix<T, 4, 4>::Zero();

        transformation_matrix.template block<3, 3>(0, 0) = getRotor().toRotationMatrix();
        transformation_matrix.template block<3, 1>(0, 3) = getTranslator().toTranslationVector();
        transformation_matrix.coeffRef(3, 3)             = 1.0;

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
        return Motor<T>({ TypeTraits<T>::One(),
                          TypeTraits<T>::Zero(),
                          TypeTraits<T>::Zero(),
                          TypeTraits<T>::Zero(),
                          TypeTraits<T>::Zero(),
                          TypeTraits<T>::Zero(),
                          TypeTraits<T>::Zero(),
                          TypeTraits<T>::Zero() });
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