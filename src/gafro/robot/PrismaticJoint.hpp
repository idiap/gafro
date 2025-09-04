// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/Joint.hpp>

namespace gafro
{

    template <class T>
    class PrismaticJoint : public Joint<T>
    {
      public:
        using Axis = typename Translator<T>::Generator;

        PrismaticJoint();

        PrismaticJoint(const std::array<T, 3> &parameters);

        PrismaticJoint(const std::array<T, 6> &parameters, int axis);

        virtual ~PrismaticJoint();

        void setAxis(const Axis &axis);

        const Axis &getAxis() const;

        Motor<T> getMotor(const T &displacement) const;

        Motor<T> getMotorDerivative(const T &angle) const;

        Translator<T> getTranslator(const T &displacement) const;

        typename Motor<T>::Generator getCurrentAxis(const Motor<T> &motor) const;

        std::unique_ptr<Joint<T>> copy() const;

      protected:
      private:
        Axis axis_;
    };

}  // namespace gafro