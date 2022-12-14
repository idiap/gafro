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

#include <gafro/algebra/Line.hpp>
#include <gafro/algebra/Point.hpp>

namespace gafro
{

    template <class T>
    Line<T>::Line(const Base &other) : Base(other)
    {}

    template <class T>
    Line<T>::Line(const Point<T> &p1, const Point<T> &p2) : Line(p1 ^ p2 ^ Ei<T>(1.0))
    {}

    // template <class T>
    // Point<T> Line<T>::getIntersectionPoint(const Line<T> &other)
    // {
    //     Base l1 = mv().normalized();
    //     Base l2 = other.mv().normalized();

    //     Base line = l1 - l2 * l1 * l2;

    //     // arbitrary point
    //     Base point = Point<T>().mv();

    //     Base point_reflected = line * point * line;

    //     Base midpoint = T(0.5) * (point + point_reflected);

    //     Base midpoint_reflected = l2 * midpoint * l2;

    //     midpoint = T(0.5) * (midpoint + midpoint_reflected);

    //     return Point<T>(T(T(-0.5) / (midpoint | blades::Ei<T>()).squared()) * (midpoint * blades::Ei<T>() * midpoint));
    // }

}  // namespace gafro