// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/FreeJoint.hpp>

namespace gafro
{

    template <class T>
    FreeJoint<T>::FreeJoint()
      : Joint<T>(Joint<T>::Type::FREE)
    {}

    template <class T>
    FreeJoint<T>::~FreeJoint() = default;

    template <class T>
    Motor<T> FreeJoint<T>::getMotor(const T &) const
    {
        return this->getFrame();
    }

    template <class T>
    Motor<T> FreeJoint<T>::getMotorDerivative(const T &) const
    {
        return this->getFrame();
    }

    template <class T>
    typename Motor<T>::Generator FreeJoint<T>::getCurrentAxis(const Motor<T> &) const
    {
        return typename Motor<T>::Generator(Motor<T>::Generator::Parameters::Zero());
    }

    template <class T>
    std::unique_ptr<Joint<T>> FreeJoint<T>::copy() const
    {
        std::unique_ptr<Joint<T>> joint = std::make_unique<FreeJoint<T>>();

        joint->setName(this->getName());
        joint->setFrame(this->getFrame());
        joint->setLimits(this->getLimits());

        return joint;
    }

}  // namespace gafro