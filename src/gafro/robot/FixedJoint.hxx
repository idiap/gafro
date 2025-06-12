// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/FixedJoint.hpp>

namespace gafro
{

    template <class T>
    FixedJoint<T>::FixedJoint() : Joint<T>(Joint<T>::Type::FIXED)
    {}

    template <class T>
    FixedJoint<T>::~FixedJoint() = default;

    template <class T>
    Motor<T> FixedJoint<T>::getMotor(const T &) const
    {
        return this->getFrame();
    }

    template <class T>
    Motor<T> FixedJoint<T>::getMotorDerivative(const T &) const
    {
        return this->getFrame();
    }

    template <class T>
    typename Motor<T>::Generator FixedJoint<T>::getCurrentAxis(const Motor<T> &) const
    {
        return typename Motor<T>::Generator(Motor<T>::Generator::Parameters::Zero());
    }

}  // namespace gafro