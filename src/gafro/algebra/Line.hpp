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

#include <gafro/algebra/Blades.hpp>
#include <gafro/algebra/Multivector.hpp>

namespace gafro
{
    template <class T>
    class Point;

    template <class T>
    class Motor;

    template <class T>
    class Line : public Multivector<T, blades::e12i, blades::e13i, blades::e23i, blades::e01i, blades::e02i, blades::e03i>
    {
      public:
        using Base = Multivector<T, blades::e12i, blades::e13i, blades::e23i, blades::e01i, blades::e02i, blades::e03i>;

        using Base::Base;

        Line(const Base &other);

        Line(const Point<T> &p1, const Point<T> &p2);

        virtual ~Line() = default;

        Motor<T> getMotor(const Line &other) const;

      protected:
      private:
      public:
        static Line X();

        static Line Y();

        static Line Z();

        static Line Random();
    };

}  // namespace gafro