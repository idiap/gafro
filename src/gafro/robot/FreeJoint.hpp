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
    class FreeJoint : public Joint<T>
    {
      public:
        FreeJoint();

        virtual ~FreeJoint();

        Motor<T> getMotor(const T &p) const;

        Motor<T> getMotorDerivative(const T &angle) const;

        typename Motor<T>::Generator getCurrentAxis(const Motor<T> &motor) const;

        std::unique_ptr<Joint<T>> copy() const;

      protected:
      private:
    };

}  // namespace gafro

#include <gafro/robot/FreeJoint.hxx>