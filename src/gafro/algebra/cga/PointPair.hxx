// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Point.hxx>
#include <gafro/algebra/cga/PointPair.hpp>

namespace gafro
{

    template <class T>
    static Point<T> split(const PointPair<T> &pp, bool bSecond)
    {
        auto point = bSecond ? (pp + Scalar<T>(pp.norm())).evaluate() : (pp - Scalar<T>(pp.norm())).evaluate();

        auto inverse = (Ei<T>(T(-1.0)) | pp).evaluate().inverse().evaluate();

        return point * inverse;
    }

    template <class T>
    PointPair<T>::PointPair() : PointPair<T>(Point<T>(), Point<T>())
    {}

    template <class T>
    PointPair<T>::PointPair(const Base &other) : Base(other)
    {}

    template <class T>
    PointPair<T>::PointPair(const PointPair &other) : Base(other)
    {}

    template <class T>
    PointPair<T>::PointPair(const Point<T> &p1, const Point<T> &p2) : Base(p1 ^ p2)
    {}

    template <class T>
    PointPair<T>::~PointPair()
    {}

    template <class T>
    Point<T> PointPair<T>::getPoint1() const
    {
        return split(*this, false);
    }

    template <class T>
    Point<T> PointPair<T>::getPoint2() const
    {
        return split(*this, true);
    }

    // template <class T>
    // Sphere<T> PointPair<T>::getSphere() const
    // {
    //     return ((mv() | (mv() ^ blades::Ei<T>()).inverse())).dual();
    // }

    // template <class T>
    // Plane<T> PointPair<T>::getPlane() const
    // {
    //     return (mv().dual() ^ blades::Ei<T>());
    // }

    // template <class T>
    // Line<T> PointPair<T>::getLine() const
    // {
    //     return (mv() ^ blades::Ei<T>());
    // }

}  // namespace gafro