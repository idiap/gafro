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
    Line<T>::Line(const Point<T> &p1, const Point<T> &p2) : Line(p1 ^ p2 ^ Ei<T>(T(1.0)))
    {}

    template <class T>
    Line<T> Line<T>::X()
    {
        return Line(Point<T>(), Point<T>(1.0, 0.0, 0.0));
    }

    template <class T>
    Line<T> Line<T>::Y()
    {
        return Line(Point<T>(), Point<T>(0.0, 1.0, 0.0));
    }

    template <class T>
    Line<T> Line<T>::Z()
    {
        return Line(Point<T>(), Point<T>(0.0, 0.0, 1.0));
    }

    template <class T>
    Line<T> Line<T>::Random()
    {
        return Line(Point<T>::Random(), Point<T>::Random());
    }

    template <class T>
    Motor<T> Line<T>::getMotor(const Line &other) const
    {
        auto l2l1 = (gafro::Scalar<double>(1.0) + other * (*this)).evaluate();
        auto l1l2 = (gafro::Scalar<double>(1.0) + (*this) * other).evaluate();

        auto k = (l1l2 + l2l1).evaluate();

        auto k0 = gafro::Scalar<double>(k.template get<gafro::blades::scalar>());
        auto k4 = (k - k0).evaluate();

        auto a1 = gafro::Scalar<double>(sqrt(1.0 / k.template get<gafro::blades::scalar>()));
        auto a2 = (gafro::Scalar<double>(1.0) - (k4 * gafro::Scalar<double>(1.0 / (2.0 * k.template get<gafro::blades::scalar>())))).evaluate();

        return Motor<T>(a1 * a2 * l2l1);
    }

}  // namespace gafro