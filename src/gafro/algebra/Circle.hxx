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

#include <gafro/algebra/Circle.hpp>
#include <gafro/algebra/Plane.hpp>
#include <gafro/algebra/Point.hpp>

namespace gafro
{

    template <typename T>
    Circle<T>::Circle(const Base &other) : Base(other)
    {}

    template <typename T>
    Circle<T>::Circle(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3) : Circle(p1 ^ p2 ^ p3)
    {}

    template <typename T>
    Point<T> Circle<T>::getCenter() const
    {
        Point<T> center((*this) * Ei<T>(T(1.0)) * (*this));

        center = center * Scalar<T>(T(1.0) / center.template get<blades::e0>());

        return center;
    }

    template <typename T>
    T Circle<T>::getRadius() const
    {
        auto mv = ((*this) * (Scalar<T>(T(-1.0)) * ((*this).dual() | Ei<T>(T(1.0))).evaluate().inverse())).evaluate();

        return sqrt(abs((mv * mv).template get<blades::scalar>()));
    }

    template <typename T>
    Plane<T> Circle<T>::getPlane() const
    {
        return Plane<T>((*this) ^ Ei<T>(T(1.0)));
    }

    template <class T>
    Circle<T> Circle<T>::Random()
    {
        return Circle(Point<T>::Random(), Point<T>::Random(), Point<T>::Random());
    }

}  // namespace gafro