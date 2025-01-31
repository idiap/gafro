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