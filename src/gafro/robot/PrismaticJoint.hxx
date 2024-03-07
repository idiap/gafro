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
    PrismaticJoint<T>::PrismaticJoint() : Joint<T>(Joint<T>::Type::PRISMATIC)
    {}

    template <class T>
    PrismaticJoint<T>::PrismaticJoint(const std::array<T, 3> &parameters) : Joint<T>(Joint<T>::Type::PRISMATIC)
    {
        throw std::runtime_error("PrismaticJoint<T>::PrismaticJoint(const std::array<T, 3> &parameters) not implemented!");

        // Rotor<T> rotor(typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(),
        // TypeTraits<T>::Zero() })), parameters[2]);

        // frame_ = Motor<T>(rotor, Translator<T>(typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ parameters[0], 0.0, parameters[1] }))));

        // axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() }));
    }

    template <class T>
    PrismaticJoint<T>::PrismaticJoint(const std::array<T, 6> &parameters, int axis) : Joint<T>(Joint<T>::Type::PRISMATIC)
    {
        Translator<T> t(typename Translator<T>::Generator({ parameters[0], parameters[1], parameters[2] }));

        Rotor<T> r1(typename Rotor<T>::Generator({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }), parameters[3]);
        Rotor<T> r2(typename Rotor<T>::Generator({ TypeTraits<T>::Zero(), TypeTraits<T>::One(), TypeTraits<T>::Zero() }), parameters[4]);
        Rotor<T> r3(typename Rotor<T>::Generator({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() }), parameters[5]);

        this->setFrame(t * r1 * r2 * r3);

        switch (axis)
        {
        case 1:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }));
            break;
        case 2:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::One(), TypeTraits<T>::Zero() }));
            break;
        case 3:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() }));
            break;
        case -1:
            axis_ =
              typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ -TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }));
            break;
        case -2:
            axis_ =
              typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), -TypeTraits<T>::One(), TypeTraits<T>::Zero() }));
            break;
        case -3:
            axis_ =
              typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), -TypeTraits<T>::One() }));
            break;
        default:
            axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() }));
        }
    }

    template <class T>
    PrismaticJoint<T>::~PrismaticJoint() = default;

    template <class T>
    void PrismaticJoint<T>::setAxis(const Axis &axis)
    {
        axis_ = axis;
    }

    template <class T>
    const typename PrismaticJoint<T>::Axis &PrismaticJoint<T>::getAxis() const
    {
        return axis_;
    }

    template <class T>
    Motor<T> PrismaticJoint<T>::getMotor(const T &displacement) const
    {
        return this->getFrame() * (Scalar<T>(TypeTraits<T>::One()) + Scalar<T>(TypeTraits<T>::Value(-0.5) * displacement) * axis_);
    }

    template <class T>
    Motor<T> PrismaticJoint<T>::getMotorDerivative(const T &displacement) const
    {
        throw std::runtime_error("PrismaticJoint<T>::getMotorDerivative not implemented!");

        return this->getFrame() * (Scalar<T>(TypeTraits<T>::One()) + Scalar<T>(TypeTraits<T>::Value(-0.5) * displacement) * axis_);
    }

    template <class T>
    Translator<T> PrismaticJoint<T>::getTranslator(const T &displacement) const
    {
        return Translator<T>(Scalar<T>(TypeTraits<T>::One()) + Scalar<T>(TypeTraits<T>::Value(-0.5) * displacement) * axis_);
    }

    template <class T>
    typename Motor<T>::Generator PrismaticJoint<T>::getCurrentAxis(const Motor<T> &motor) const
    {
        auto expression = motor * axis_ * motor.reverse();

        return (typename Rotor<T>::Generator()                    //
                + E1i<T>(expression.template get<blades::e1i>())  //
                + E2i<T>(expression.template get<blades::e2i>())  //
                + E3i<T>(expression.template get<blades::e3i>()))
          .evaluate();
    }

}  // namespace gafro