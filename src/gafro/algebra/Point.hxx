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

#include <gafro/algebra/Point.hpp>

namespace gafro
{

    template <class T>
    Point<T>::Point() : Point<T>(0.0, 0.0, 0.0)
    {}

    template <class T>
    Point<T>::Point(const T &x, const T &y, const T &z)
    {
        this->template set<blades::e1>(x);
        this->template set<blades::e2>(y);
        this->template set<blades::e3>(z);
        this->template set<blades::ei>(T(0.5) * (x * x + y * y + z * z));
        this->template set<blades::e0>(T(1.0));
    }

    template <class T>
    Point<T>::Point(const Base &other) : Base(other)
    {}

    template <class T>
    Point<T> Point<T>::X(const T &x)
    {
        return Point(x, T(0.0), T(0.0));
    }

    template <class T>
    Point<T> Point<T>::Y(const T &y)
    {
        return Point(T(0.0), y, T(0.0));
    }

    template <class T>
    Point<T> Point<T>::Z(const T &z)
    {
        return Point(T(0.0), T(0.0), z);
    }

    template <class T>
    Point<T> Point<T>::Random()
    {
        Eigen::Vector3<T> p = Eigen::Vector3<T>::Random();

        return Point(p.x(), p.y(), p.z());
    }

}  // namespace gafro