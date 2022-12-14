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

#include <gafro/robot/Joint.hpp>

namespace gafro
{

    template <class T>
    Joint<T>::Joint()
    {}

    template <class T>
    Joint<T>::Joint(const std::array<T, 3> &parameters)
    {
        Rotor<T> rotor(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(1.0), T(0.0), T(0.0) })), parameters[2]);

        frame_ = Motor<T>(rotor, Translator<T>(typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ parameters[0], 0.0, parameters[1] }))));

        axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(0.0), T(1.0) }));
    }

    template <class T>
    Joint<T>::Joint(const std::array<T, 6> &parameters, int axis)
    {
        Translator<T> t({ parameters[0], parameters[1], parameters[2] });

        Rotor<T> r1(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(1.0), T(0.0), T(0.0) })), parameters[3]);
        Rotor<T> r2(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(1.0), T(0.0) })), parameters[4]);
        Rotor<T> r3(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(0.0), T(1.0) })), parameters[5]);

        frame_ = Motor<T>(t) * Motor<T>(r1) * Motor<T>(r2) * Motor<T>(r3);

        switch (axis)
        {
        case 1:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(1.0), T(0.0), T(0.0) }));
            break;
        case 2:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(1.0), T(0.0) }));
            break;
        case 3:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(0.0), T(1.0) }));
            break;
        case -1:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(-1.0), T(0.0), T(0.0) }));
            break;
        case -2:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(-1.0), T(0.0) }));
            break;
        case -3:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(0.0), T(-1.0) }));
            break;
        default:
            axis_ = typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(0.0), T(1.0) }));
        }
    }

    template <typename T>
    template <typename T2>
    Joint<T>::Joint(const Joint<T2> &other) : frame_(other.getFrame())
    {}

    template <class T>
    Joint<T>::~Joint()
    {}

    template <class T>
    const Motor<T> &Joint<T>::getFrame() const
    {
        return frame_;
    }

    template <class T>
    const typename Rotor<T>::Generator &Joint<T>::getAxis() const
    {
        return axis_;
        // return typename Motor<T>::Generator(axis_.vector(), Eigen::Matrix<T, 3, 1>::Zero());
    }

    template <class T>
    Motor<T> Joint<T>::getMotor(const T &angle) const
    {
        T half_angle = 0.5 * angle;

        T s = -sin(half_angle);

        return frame_ * gafro::Rotor<T>({ cos(half_angle),                        //
                                          s * axis_.template get<blades::e23>(),  //
                                          s * axis_.template get<blades::e13>(),  //
                                          s * axis_.template get<blades::e12>() });
        // (Scalar<T>(cos(half_angle)) + Scalar<T>(-sin(half_angle)) * axis_);
    }

    template <class T>
    auto Joint<T>::getMotorExpression(const T &angle) const
    {
        return frame_ * (Scalar<T>(cos(0.5 * angle)) + Scalar<T>(-sin(0.5 * angle)) * axis_);
    }

    template <class T>
    Rotor<T> Joint<T>::getRotor(const T &angle) const
    {
        return Rotor<T>(axis_, angle);
    }

    template <class T>
    const typename Rotor<T>::Generator &Joint<T>::getRotationRotorGenerator() const
    {
        return axis_;
    }

}  // namespace gafro