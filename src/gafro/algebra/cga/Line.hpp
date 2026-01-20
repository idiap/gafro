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
    class Motor;

    template <class T>
    class Line : public Multivector<T, blades::e01i, blades::e02i, blades::e12i, blades::e03i, blades::e13i, blades::e23i>
    {
      public:
        using Base = Multivector<T, blades::e01i, blades::e02i, blades::e12i, blades::e03i, blades::e13i, blades::e23i>;

        using Base::Base;

        Line(const Base &other);

        Line(const Point<T> &p1, const Point<T> &p2);

        virtual ~Line() = default;

        Motor<T> getMotor(const Line &other) const;

        std::pair<Point<T>, Point<T>> computeClosestPoints(const Line &other) const;

        Line<T> computeOrthogonalLine(const Line &other) const;

      protected:
      private:
      public:
        static Line X();

        static Line Y();

        static Line Z();

        static Line Random();
    };

}  // namespace gafro