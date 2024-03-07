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

#include <gafro/algebra/Sphere.hpp>
#include <gafro/algebra/expressions/Reflection.hpp>

namespace gafro
{

    template <typename T>
    Sphere<T>::Sphere() : Base()
    {
        this->template set<blades::e0>(1.0);
    }

    template <typename T>
    Sphere<T>::Sphere(const Base &other) : Base(other)
    {}

    template <typename T>
    Sphere<T>::Sphere(const Point<T> &center, const T &radius)  //
      : Base((center + Ei<T>(-0.5 * radius * radius)).evaluate().dual())
    {}

    template <typename T>
    Sphere<T>::Sphere(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3, const Point<T> &p4)  //
      : Base(p1 ^ p2 ^ p3 ^ p4)
    {}

    template <typename T>
    T Sphere<T>::getRadius() const
    {
        return sqrt(abs(((*this) * Scalar<T>(-1.0 / (this->dual() | Ei<T>(1.0)).template get<blades::scalar>())).evaluate().squaredNorm()));
    }

    template <typename T>
    Point<T> Sphere<T>::getCenter() const
    {
        auto dual_sphere = this->dual().evaluate();

        const typename Point<T>::Base ei({ 0.0, 0.0, 0.0, 1.0, 0.0 });

        Point<T> center = Reflection<typename Point<T>::Base, typename Dual<Sphere>::Type>(ei, dual_sphere).evaluate();
        center = (center * Scalar<T>(1.0 / center.template get<blades::e0>())).evaluate();

        return center;
    }

    template <class T>
    Sphere<T> Sphere<T>::Random()
    {
        return Sphere(Point<T>::Random(), Point<T>::Random(), Point<T>::Random(), Point<T>::Random());
    }

}  // namespace gafro