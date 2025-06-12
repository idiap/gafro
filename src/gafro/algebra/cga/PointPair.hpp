// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/cga/Blades.hpp>

namespace gafro
{
    template <class T>
    class Point;

    template <class T>
    class PointPair : public Multivector<T, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23, blades::e0i, blades::e1i,
                                         blades::e2i, blades::e3i>
    {
      public:
        using Base = Multivector<T, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23, blades::e0i, blades::e1i,
                                 blades::e2i, blades::e3i>;

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