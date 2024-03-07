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

#include <gafro/algebra/Blades.hpp>
#include <gafro/algebra/Multivector.hpp>

namespace gafro
{
    template <typename T>
    class Point;
    // template <typename T>
    // class Circle;

    template <typename T>
    class Sphere : public Multivector<T, blades::e123i, blades::e0123, blades::e012i, blades::e023i, blades::e013i>
    {
      public:
        using Base = Multivector<T, blades::e123i, blades::e0123, blades::e012i, blades::e023i, blades::e013i>;

        using Base::Base;

        Sphere();

        Sphere(const Base &other);

        Sphere(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3, const Point<T> &p4);

        Sphere(const Point<T> &center, const T &radius);

        virtual ~Sphere() = default;

        T getRadius() const;

        Point<T> getCenter() const;

      protected:
      private:
      public:
        static Sphere Random();
    };

}  // namespace gafro