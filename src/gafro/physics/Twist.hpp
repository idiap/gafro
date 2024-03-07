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

#include <gafro/algebra.hpp>

namespace gafro
{

    template <class T>
    class Wrench;

    template <class T>
    class Twist : public Multivector<T, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i>
    {
      public:
        using Base = Multivector<T, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i>;
        using Parameters = typename Base::Parameters;

        using Angular = Multivector<T, blades::e23, blades::e13, blades::e12>;
        using Linear = Multivector<T, blades::e1i, blades::e2i, blades::e3i>;

        using Base::Base;
        using Base::vector;

        Twist();

        Twist(const Base &multivector);

        Twist(const Parameters &multivector);

        virtual ~Twist();

        Base &multivector();

        const Base &multivector() const;

        Angular getAngular() const;

        Linear getLinear() const;

        Twist transform(const Motor<T> &motor) const;

        Wrench<T> commute(const Wrench<T> &wrench) const;

        template <class E>
        Twist &operator=(const Expression<E, Base> &expression);

        Twist &operator+=(const Twist<T> &twist);

      protected:
      private:
    };

}  // namespace gafro