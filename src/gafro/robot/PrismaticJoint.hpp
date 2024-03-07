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

      protected:
      private:
        Axis axis_;
    };

}  // namespace gafro