// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Line.hpp>
#include <gafro/algebra/cga/Point.hpp>
#include <gafro/algebra/cga/Rotor.hpp>
#include <gafro/algebra/cga/Vector.hpp>

namespace gafro
{

    template <class T>
    Line<T>::Line(const Base &other)
      : Base(other)
    {}

    template <class T>
    Line<T>::Line(const Point<T> &p1, const Point<T> &p2)
      : Line(p1 ^ p2 ^ Ei<T>(T(1.0)))
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

        auto l2l1 = (Scalar<T>(TypeTraits<T>::Value(1.0)) + l2 * l1).evaluate();
        auto l1l2 = (Scalar<T>(TypeTraits<T>::Value(1.0)) + l1 * l2).evaluate();

        auto k = (l1l2 + l2l1).evaluate();

        auto k0 = Scalar<T>(k.template get<blades::scalar>());
        auto k4 = (k - k0).evaluate();

        if (k.template get<blades::scalar>() == TypeTraits<T>::Zero())
        {
            return Motor<T>(Scalar<T>(TypeTraits<T>::Value(1.0)) - Scalar<T>(TypeTraits<T>::Value(0.5)) * l2l1);
        }

        auto a1 = Scalar<T>(sqrt(TypeTraits<T>::Value(1.0) / k.template get<blades::scalar>()));

        auto a2 = (Scalar<T>(TypeTraits<T>::Value(1.0)) -
                   (k4 * Scalar<T>(TypeTraits<T>::Value(1.0) / (TypeTraits<T>::Value(2.0) * k.template get<blades::scalar>()))))
                    .evaluate();

        auto a = (a1 * a2).evaluate();

        return Motor<T>(a * l2l1);
    }

    template <class T>
    std::pair<Point<T>, Point<T>> Line<T>::computeClosestPoints(const Line &other) const
    {
        static auto cross = [](auto a, auto b) { return 0.5 * (a * b - b * a); };
        static auto dot   = [](auto a, auto b) { return -0.5 * (a * b + b * a); };

        Line<T> l1 = this->normalized();
        Line<T> l2 = other.normalized();

        typename Rotor<T>::Generator l1p = (l1 | E0i<T>::One()) * E123<T>::One();
        typename Rotor<T>::Generator l1d = (l1.dual() | E0<T>::One()) * E123<T>::One();
        typename Rotor<T>::Generator l2p = (l2 | E0i<T>::One()) * E123<T>::One();
        typename Rotor<T>::Generator l2d = (l2.dual() | E0<T>::One()) * E123<T>::One();

        Vector<T> p1vec =
          ((cross(-l1d, cross(l2p, cross(l1p, l2p))) + dot(l2d, cross(l1p, l2p)) * l1p) * E123<T>(-1.0 / cross(l1p, l2p).squaredNorm()));

        Vector<T> p2vec =
          ((cross(l2d, cross(l1p, cross(l1p, l2p))) - dot(l1d, cross(l1p, l2p)) * l2p) * E123<T>(-1.0 / cross(l1p, l2p).squaredNorm()));

        Point<T> p1(p1vec.vector()[0], p1vec.vector()[1], p1vec.vector()[2]);
        Point<T> p2(p2vec.vector()[0], p2vec.vector()[1], p2vec.vector()[2]);

        return std::make_pair(p1, p2);
    }

    template <class T>
    Line<T> Line<T>::computeOrthogonalLine(const Line &other) const
    {
        auto [p1, p2] = computeClosestPoints(other);

        return Line(p1, p2);
    }

}  // namespace gafro