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

#include <gafro/robot/Manipulator.hpp>

namespace gafro
{
    namespace core
    {
        template <class T>
        class UniversalRobot10 : public Manipulator<T, 6>
        {
          public:
            UniversalRobot10();

            virtual ~UniversalRobot10();

          protected:
          private:
        };

        template <class T>
        UniversalRobot10<T>::UniversalRobot10()  //
          : Manipulator<T, 6>(
              {
                std::array<T, 6>({ 0.0, 0.0, 0.1273, 0.0, 0.0, 0.0 }),            //
                std::array<T, 6>({ 0.0, 0.220941, 0.0, 0.0, -M_PI / 2.0, 0.0 }),  //
                std::array<T, 6>({ 0.0, -0.1719, 0.612, 0.0, 0.0, 0.0 }),         //
                std::array<T, 6>({ 0.0, 0.0, 0.5723, 0.0, -M_PI / 2.0, 0.0 }),    //
                std::array<T, 6>({ 0.0, 0.1149, 0.0, 0.0, 0.0, 0.0 }),            //
                std::array<T, 6>({ 0.0, 0.0, 0.1157, 0.0, 0.0, 0.0 })             //
              },
              std::array<int, 6>({ 3, -2, -2, -2, 3, -2 }))
        {}

        template <class T>
        UniversalRobot10<T>::~UniversalRobot10()
        {}

    }  // namespace core

    using UniversalRobot10 = core::UniversalRobot10<double>;

}  // namespace gafro