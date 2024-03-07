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
    class Wrench : public Multivector<T, blades::e23, blades::e13, blades::e12, blades::e01, blades::e02, blades::e03>
    {
      public:
        using Base = Multivector<T, blades::e23, blades::e13, blades::e12, blades::e01, blades::e02, blades::e03>;
        using Parameters = typename Base::Parameters;

        using Base::Base;
        using Base::vector;

        Wrench();

        Wrench(const Base &multivector);

        Wrench(const Parameters &multivector);

        virtual ~Wrench();

        Base &multivector();

        const Base &multivector() const;

        Wrench transform(const Motor<T> &motor) const;

        template <class E>
        Wrench &operator=(const Expression<E, Base> &expression);

        Wrench &operator+=(const Wrench<T> &wrench);

        Wrench &operator-=(const Wrench<T> &wrench);

      protected:
      private:
      public:
        static Wrench Zero();
    };

    template <class T>
    Wrench<T> operator-(const Wrench<T> &wrench1, const Wrench<T> &wrench2);

}  // namespace gafro