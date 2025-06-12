// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Line.hpp>
#include <gafro/algebra/cga/Point.hpp>

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
        Line<T> l1 = this->normalized();
        Line<T> l2 = other.normalized();

        auto l2l1 = (gafro::Scalar<double>(1.0) + l2 * l1).evaluate();
        auto l1l2 = (gafro::Scalar<double>(1.0) + l1 * l2).evaluate();

        auto k = (l1l2 + l2l1).evaluate();

        auto k0 = gafro::Scalar<double>(k.template get<gafro::blades::scalar>());
        auto k4 = (k - k0).evaluate();

        auto a1 = gafro::Scalar<double>(sqrt(1.0 / k.template get<gafro::blades::scalar>()));
        auto a2 = (gafro::Scalar<double>(1.0) - (k4 * gafro::Scalar<double>(1.0 / (2.0 * k.template get<gafro::blades::scalar>())))).evaluate();

        return Motor<T>(a1 * a2 * l2l1);
    }

}  // namespace gafro