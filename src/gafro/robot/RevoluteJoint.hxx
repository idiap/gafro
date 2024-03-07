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

#include <gafro/robot/Joint.hxx>
#include <gafro/robot/RevoluteJoint.hpp>

namespace gafro
{

    template <class T>
    RevoluteJoint<T>::RevoluteJoint() : Joint<T>(Joint<T>::Type::REVOLUTE)
    {}

    template <class T>
    RevoluteJoint<T>::RevoluteJoint(const std::array<T, 3> &parameters) : Joint<T>(Joint<T>::Type::REVOLUTE)
    {
        Rotor<T> rotor(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() })),
                       parameters[2]);

        axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() }));

        this->setFrame(Motor<T>(
          rotor, Translator<T>(typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ parameters[0], TypeTraits<T>::Zero(), parameters[1] })))));
    }

    template <class T>
    RevoluteJoint<T>::RevoluteJoint(const std::array<T, 6> &parameters, int axis) : Joint<T>(Joint<T>::Type::REVOLUTE)
    {
        Translator<T> t(typename Translator<T>::Generator({ parameters[0], parameters[1], parameters[2] }));

        Rotor<T> r1(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() })),
                    parameters[3]);
        Rotor<T> r2(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::One(), TypeTraits<T>::Zero() })),
                    parameters[4]);
        Rotor<T> r3(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() })),
                    parameters[5]);

        this->setFrame(Motor<T>(t) * Motor<T>(r1) * Motor<T>(r2) * Motor<T>(r3));

        switch (axis)
        {
        case 1:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }));
            break;
        case 2:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::One(), TypeTraits<T>::Zero() }));
            break;
        case 3:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() }));
            break;
        case -1:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ -TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }));
            break;
        case -2:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), -TypeTraits<T>::One(), TypeTraits<T>::Zero() }));
            break;
        case -3:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), -TypeTraits<T>::One() }));
            break;
        default:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() }));
        }
    }

    template <class T>
    RevoluteJoint<T> &RevoluteJoint<T>::operator=(RevoluteJoint &&other)
    {
        axis_ = std::move(other.axis_);
        Joint<T>::operator=(std::move(other));

        return *this;
    }

    template <class T>
    RevoluteJoint<T>::~RevoluteJoint()
    {}

    template <class T>
    void RevoluteJoint<T>::setAxis(const Axis &axis)
    {
        axis_ = axis;
    }

    template <class T>
    const typename Rotor<T>::Generator &RevoluteJoint<T>::getAxis() const
    {
        return axis_;
    }

    template <class T>
    Motor<T> RevoluteJoint<T>::getMotor(const T &angle) const
    {
        T half_angle = 0.5 * angle;

        T s = -sin(half_angle);

        return this->getFrame() * gafro::Rotor<T>({ cos(half_angle),                        //
                                                    s * axis_.template get<blades::e23>(),  //
                                                    s * axis_.template get<blades::e13>(),  //
                                                    s * axis_.template get<blades::e12>() });
    }

    template <class T>
    Motor<T> RevoluteJoint<T>::getMotorDerivative(const T &angle) const
    {
        return this->getFrame() * Scalar<T>(TypeTraits<T>::Value(-0.5)) * this->getAxis() * this->getRotor(angle);
    }

    template <class T>
    Rotor<T> RevoluteJoint<T>::getRotor(const T &angle) const
    {
        return Rotor<T>(axis_, angle);
    }

    template <class T>
    typename Motor<T>::Generator RevoluteJoint<T>::getCurrentAxis(const Motor<T> &motor) const
    {
        return (motor * axis_ * motor.reverse()).template evaluateAs<typename Motor<T>::Generator>();
    }

}  // namespace gafro