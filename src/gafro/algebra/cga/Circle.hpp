// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/ConformalTransformation.hpp>
#include <gafro/algebra/cga/SimilarityTransformation.hpp>

namespace gafro
{
    template <class T>
    class Point;

    template <class T>
    class Plane;

    template <class T>
    class Motor;

    template <class T>
    class Circle : public Multivector<T, blades::e012, blades::e013, blades::e023, blades::e123, blades::e01i, blades::e02i, blades::e12i,
                                      blades::e03i, blades::e13i, blades::e23i>
    {
      public:
        using Base = Multivector<T, blades::e012, blades::e013, blades::e023, blades::e123, blades::e01i, blades::e02i, blades::e12i, blades::e03i,
                                 blades::e13i, blades::e23i>;

        using Base::Base;

        Circle(const Base &other);

        Circle(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3);

        virtual ~Circle() = default;

        Plane<T> getPlane() const;

        Point<T> getCenter() const;

        T getRadius() const;

        Motor<T> getMotor(const Circle &target) const;

        SimilarityTransformation<T> getSimilarityTransformation(const Circle &target) const;

        ConformalTransformation<T> getConformalTransformation(const Circle &target) const;

      protected:
      private:
      public:
        static Circle Random();

        static Circle Unit(const Motor<T> &motor, const T &radius = TypeTraits<T>::Value(1.0));
    };

}  // namespace gafro