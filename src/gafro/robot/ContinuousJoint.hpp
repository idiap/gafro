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
    class ContinuousJoint : public Joint<T>
    {
      public:
        using Axis = typename Rotor<T>::Generator;

        ContinuousJoint();

        ContinuousJoint(const std::array<T, 3> &parameters);

        ContinuousJoint(const std::array<T, 6> &parameters, int axis);

        ContinuousJoint &operator=(ContinuousJoint &&other);

        virtual ~ContinuousJoint();

        void setAxis(const Axis &axis);

        Motor<T> getMotor(const T &angle) const;

        Motor<T> getMotorDerivative(const T &angle) const;

        Rotor<T> getRotor(const T &angle) const;

        const typename Rotor<T>::Generator &getAxis() const;

        typename Motor<T>::Generator getCurrentAxis(const Motor<T> &motor) const;

        std::unique_ptr<Joint<T>> copy() const;

      protected:
      private:
        Axis axis_;
    };

}  // namespace gafro