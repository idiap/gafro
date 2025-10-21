// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/ContinuousJoint.hpp>
#include <gafro/robot/Joint.hxx>

namespace gafro
{

    template <class T>
    ContinuousJoint<T>::ContinuousJoint()
      : Joint<T>(Joint<T>::Type::CONTINUOUS)
    {}

    template <class T>
    ContinuousJoint<T>::ContinuousJoint(const std::array<T, 3> &parameters)
      : Joint<T>(Joint<T>::Type::CONTINUOUS)
    {
        Rotor<T> rotor(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() })),
                       parameters[2]);

        axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }));

        this->setFrame(Motor<T>(
          rotor, Translator<T>(typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ parameters[0], TypeTraits<T>::Zero(), parameters[1] })))));
    }

    template <class T>
    ContinuousJoint<T>::ContinuousJoint(const std::array<T, 6> &parameters, int axis)
      : Joint<T>(Joint<T>::Type::CONTINUOUS)
    {
        Translator<T> t(typename Translator<T>::Generator({ parameters[0], parameters[1], parameters[2] }));

        Rotor<T> r1(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() })),
                    parameters[5]);
        Rotor<T> r2(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::One(), TypeTraits<T>::Zero() })),
                    parameters[4]);
        Rotor<T> r3(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() })),
                    parameters[3]);

        this->setFrame(Motor<T>(t) * Motor<T>(r1) * Motor<T>(r2) * Motor<T>(r3));

        switch (axis)
        {
        case 1:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() }));
            break;
        case 2:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::One(), TypeTraits<T>::Zero() }));
            break;
        case 3:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }));
            break;
        case -1:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), -TypeTraits<T>::One() }));
            break;
        case -2:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), -TypeTraits<T>::One(), TypeTraits<T>::Zero() }));
            break;
        case -3:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ -TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }));
            break;
        default:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }));
        }
    }

    template <class T>
    ContinuousJoint<T> &ContinuousJoint<T>::operator=(ContinuousJoint &&other)
    {
        axis_ = std::move(other.axis_);
        Joint<T>::operator=(std::move(other));

        return *this;
    }

    template <class T>
    ContinuousJoint<T>::~ContinuousJoint()
    {}

    template <class T>
    void ContinuousJoint<T>::setAxis(const Axis &axis)
    {
        axis_ = axis;
    }

    template <class T>
    const typename Rotor<T>::Generator &ContinuousJoint<T>::getAxis() const
    {
        return axis_;
    }

    template <class T>
    Motor<T> ContinuousJoint<T>::getMotor(const T &angle) const
    {
        T half_angle = 0.5 * angle;

        T s = -sin(half_angle);

        return this->getFrame() * gafro::Rotor<T>({ cos(half_angle),                        //
                                                    s * axis_.template get<blades::e12>(),  //
                                                    s * axis_.template get<blades::e13>(),  //
                                                    s * axis_.template get<blades::e23>() });
    }

    template <class T>
    Motor<T> ContinuousJoint<T>::getMotorDerivative(const T &angle) const
    {
        return this->getFrame() * Scalar<T>(TypeTraits<T>::Value(-0.5)) * this->getAxis() * this->getRotor(angle);
    }

    template <class T>
    Rotor<T> ContinuousJoint<T>::getRotor(const T &angle) const
    {
        return Rotor<T>(axis_, angle);
    }

    template <class T>
    typename Motor<T>::Generator ContinuousJoint<T>::getCurrentAxis(const Motor<T> &motor) const
    {
        return (motor * axis_ * motor.reverse()).template evaluateAs<typename Motor<T>::Generator>();
    }

}  // namespace gafro