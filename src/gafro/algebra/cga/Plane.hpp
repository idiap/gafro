// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Multivector.hpp>
#include <gafro/algebra/cga/Blades.hpp>

namespace gafro
{
    template <class T>
    class Point;

    template <class T>
    class Vector;

    template <class T>
    class Motor;

    template <typename T>
    class Plane : public Multivector<T, blades::e012i, blades::e013i, blades::e023i, blades::e123i>
    {
      public:
        using Base = Multivector<T, blades::e012i, blades::e013i, blades::e023i, blades::e123i>;

        using Base::Base;

        Plane(const Base &other);

        Plane(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3);

        virtual ~Plane() = default;

        Vector<T> getNormal() const;

        Motor<T> getMotor(const Plane &other) const;

      protected:
      private:
      public:
        static Plane XY(const T &z = T(0.0));

        static Plane XZ(const T &y = T(0.0));

        static Plane YZ(const T &x = T(0.0));

        static Plane Random();

        static Plane estimateFromPoints(const std::vector<Point<T>> &points);

        static Plane estimateFromPoints(const std::vector<Point<T>> &points, const Eigen::VectorXi &indices);
    };

}  // namespace gafro