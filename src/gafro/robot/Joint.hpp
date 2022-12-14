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

#include <gafro/algebra.hpp>

namespace gafro
{

    template <class T>
    class Joint
    {
      public:
        Joint();

        Joint(const std::array<T, 3> &parameters);

        Joint(const std::array<T, 6> &parameters, int axis);

        template <typename T2>
        Joint(const Joint<T2> &other);

        virtual ~Joint();

        auto getMotorExpression(const T &angle) const;

        Motor<T> getMotor(const T &angle) const;

        Rotor<T> getRotor(const T &angle) const;

        const Motor<T> &getFrame() const;

        const typename Rotor<T>::Generator &getAxis() const;

        const typename Rotor<T>::Generator &getRotationRotorGenerator() const;

      protected:
      private:
        Motor<T> frame_;

        typename Rotor<T>::Generator axis_;
    };

}  // namespace gafro