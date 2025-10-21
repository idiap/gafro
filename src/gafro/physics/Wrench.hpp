// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra.hpp>

namespace gafro
{

    template <class T>
    class Wrench : public Multivector<T, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23>
    {
      public:
        using Base = Multivector<T, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23>;
        using Linear = Multivector<T, blades::e01, blades::e02, blades::e03>;
        using Angular = Multivector<T, blades::e12, blades::e13, blades::e23>;
        using Parameters = typename Base::Parameters;

        using Base::Base;
        using Base::vector;

        Wrench();

        Wrench(const Base &multivector);

        Wrench(const T &tx, const T &ty, const T &tz, const T &fx, const T &fy, const T &fz);

        virtual ~Wrench();

        void setLinear(const Linear &linear);

        void setAngular(const Angular &angular);

        Linear getLinear() const;

        Angular getAngular() const;

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