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

#include <gafro/robot/PrismaticJoint.hpp>

namespace gafro
{

    template <class T>
    PrismaticJoint<T>::PrismaticJoint(const std::array<T, 3> &parameters)
    {
        throw std::runtime_error("PrismaticJoint<T>::PrismaticJoint(const std::array<T, 3> &parameters) not implemented!");

        // Rotor<T> rotor(typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(1.0), T(0.0), T(0.0) })), parameters[2]);

        // frame_ = Motor<T>(rotor, Translator<T>(typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ parameters[0], 0.0, parameters[1] }))));

        // axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(0.0), T(1.0) }));
    }

    template <class T>
    PrismaticJoint<T>::PrismaticJoint(const std::array<T, 6> &parameters, int axis)
    {
        Translator<T> t(typename Translator<T>::Generator({ parameters[0], parameters[1], parameters[2] }));

        Rotor<T> r1(typename Rotor<T>::Generator({ T(1.0), T(0.0), T(0.0) }), parameters[3]);
        Rotor<T> r2(typename Rotor<T>::Generator({ T(0.0), T(1.0), T(0.0) }), parameters[4]);
        Rotor<T> r3(typename Rotor<T>::Generator({ T(0.0), T(0.0), T(1.0) }), parameters[5]);

        frame_ = t * r1 * r2 * r3;

        switch (axis)
        {
        case 1:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(1.0), T(0.0), T(0.0) }));
            break;
        case 2:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(1.0), T(0.0) }));
            break;
        case 3:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(0.0), T(1.0) }));
            break;
        case -1:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(-1.0), T(0.0), T(0.0) }));
            break;
        case -2:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(-1.0), T(0.0) }));
            break;
        case -3:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(0.0), T(-1.0) }));
            break;
        default:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(0.0), T(1.0) }));
        }
    }

    template <class T>
    const Motor<T> &PrismaticJoint<T>::getFrame() const
    {
        return frame_;
    }

    template <class T>
    const typename Translator<T>::Generator &PrismaticJoint<T>::getAxis() const
    {
        return axis_;
    }

    template <class T>
    Motor<T> PrismaticJoint<T>::getMotor(const T &displacement) const
    {
        return frame_ * (Scalar<T>(T(1.0)) + Scalar<T>(T(-0.5 * displacement)) * axis_);
    }

    template <class T>
    Translator<T> PrismaticJoint<T>::getTranslator(const T &displacement) const
    {
        return Translator<T>(Scalar<T>(T(1.0)) + Scalar<T>(T(-0.5 * displacement)) * axis_);
    }

}  // namespace gafro