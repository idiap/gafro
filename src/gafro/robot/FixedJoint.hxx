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