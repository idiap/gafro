// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Multivector.hpp>
#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/SimilarityTransformation.hpp>

namespace gafro
{
    template <typename T>
    class Point;

    template <typename T>
    class Sphere : public Multivector<T, blades::e0123, blades::e012i, blades::e013i, blades::e023i, blades::e123i>
    {
      public:
        using Base = Multivector<T, blades::e0123, blades::e012i, blades::e013i, blades::e023i, blades::e123i>;

        using Base::Base;

        Sphere();

        Sphere(const Base &other);

        Sphere(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3, const Point<T> &p4);

        Sphere(const Point<T> &center, const T &radius);

        virtual ~Sphere() = default;

        T getRadius() const;

        Point<T> getCenter() const;

        SimilarityTransformation<T> getSimilarityTransformation(const Sphere &target) const;

      protected:
      private:
      public:
        static Sphere Random();

        static Sphere Unit();
    };

}  // namespace gafro