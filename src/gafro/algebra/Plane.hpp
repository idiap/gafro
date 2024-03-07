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

    template <typename T>
    class Plane : public Multivector<T, blades::e123i, blades::e012i, blades::e023i, blades::e013i>
    {
      public:
        using Base = Multivector<T, blades::e123i, blades::e012i, blades::e023i, blades::e013i>;

        using Base::Base;

        Plane(const Base &other);

        Plane(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3);

        virtual ~Plane() = default;

        Multivector<T, blades::e1, blades::e2, blades::e3> getNormal() const;

      protected:
      private:
      public:
        static Plane XY(const T &z = T(0.0));

        static Plane XZ(const T &y = T(0.0));

        static Plane YZ(const T &x = T(0.0));

        static Plane Random();
    };

}  // namespace gafro