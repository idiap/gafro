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
#include <gafro/algebra/Multivector.hxx>

namespace gafro
{
    template <class T>
    class Point;

    template <class T>
    class PointPair : public Multivector<T, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i, blades::e01, blades::e02,
                                         blades::e03, blades::e0i>
    {
      public:
        using Base = Multivector<T, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i, blades::e01, blades::e02,
                                 blades::e03, blades::e0i>;

        using Base::Base;

        PointPair();

        PointPair(const Base &other);

        PointPair(const PointPair &other);

        PointPair(const Point<T> &p1, const Point<T> &p2);

        virtual ~PointPair();

        Point<T> getPoint1() const;

        Point<T> getPoint2() const;

        // Sphere<T> getSphere() const;

        // Plane<T> getPlane() const;

        // Line<T> getLine() const;

      protected:
      private:
    };
}  // namespace gafro