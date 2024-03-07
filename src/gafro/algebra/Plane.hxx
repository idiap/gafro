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

#include <gafro/algebra/Plane.hpp>
#include <gafro/algebra/Point.hpp>

namespace gafro
{
    template <typename T>
    Plane<T>::Plane(const Base &other) : Base(other)
    {}

    template <typename T>
    Plane<T>::Plane(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3)  //
      : Base(p1 ^ p2 ^ p3 ^ Ei<T>(T(1.0)))
    {}

    template <typename T>
    Multivector<T, blades::e1, blades::e2, blades::e3> Plane<T>::getNormal() const
    {
        return this->dual() - Scalar<T>(T(0.5)) * (this->dual() | E0<T>(T(1.0))) * Ei<T>(T(1.0));
    }

    template <typename T>
    Plane<T> Plane<T>::XY(const T &z)
    {
        return Plane(Point<T>(0.0, 0.0, z), Point<T>(1.0, 0.0, z), Point<T>(0.0, 1.0, z));
    }

    template <typename T>
    Plane<T> Plane<T>::XZ(const T &y)
    {
        return Plane(Point<T>(0.0, y, 0.0), Point<T>(1.0, y, 0.0), Point<T>(0.0, y, 1.0));
    }

    template <typename T>
    Plane<T> Plane<T>::YZ(const T &x)
    {
        return Plane(Point<T>(x, 0.0, 0.0), Point<T>(x, 1.0, 0.0), Point<T>(x, 0.0, 1.0));
    }

    template <class T>
    Plane<T> Plane<T>::Random()
    {
        return Plane(Point<T>::Random(), Point<T>::Random(), Point<T>::Random());
    }

}  // namespace gafro