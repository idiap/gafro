// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/PrismaticJoint.hpp>

namespace gafro
{

    template <class T>
    PrismaticJoint<T>::PrismaticJoint()
      : Joint<T>(Joint<T>::Type::PRISMATIC)
    {}

    template <class T>
    PrismaticJoint<T>::PrismaticJoint(const std::array<T, 3> &parameters)
      : Joint<T>(Joint<T>::Type::PRISMATIC)
    {
        throw std::runtime_error("PrismaticJoint<T>::PrismaticJoint(const std::array<T, 3> &parameters) not implemented!");

        // Rotor<T> rotor(typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::One(), TypeTraits<T>::Zero(),
        // TypeTraits<T>::Zero() })), parameters[2]);

        // frame_ = Motor<T>(rotor, Translator<T>(typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ parameters[0], 0.0, parameters[1] }))));

        // axis_ = typename Translator<T>::Generator(Eigen::Matrix<T, 3, 1>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() }));
    }

    template <class T>
    PrismaticJoint<T>::PrismaticJoint(const std::array<T, 6> &parameters, int axis)
      : Joint<T>(Joint<T>::Type::PRISMATIC)
    {
        Translator<T> t(typename Translator<T>::Generator({ parameters[0], parameters[1], parameters[2] }));

        Rotor<T> r1(typename Rotor<T>::Generator({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::One() }), parameters[5]);
        Rotor<T> r2(typename Rotor<T>::Generator({ TypeTraits<T>::Zero(), TypeTraits<T>::One(), TypeTraits<T>::Zero() }), parameters[4]);
        Rotor<T> r3(typename Rotor<T>::Generator({ TypeTraits<T>::One(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero() }), parameters[3]);

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

    template <class T>
    std::unique_ptr<Joint<T>> PrismaticJoint<T>::copy() const
    {
        std::unique_ptr<Joint<T>> joint = std::make_unique<PrismaticJoint<T>>();

        joint->setName(this->getName());
        joint->setFrame(this->getFrame());
        joint->setLimits(this->getLimits());
        static_cast<PrismaticJoint<T> *>(joint.get())->setAxis(this->getAxis());

        return joint;
    }

}  // namespace gafro