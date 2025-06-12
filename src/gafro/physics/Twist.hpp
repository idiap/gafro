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
    class Wrench;

    template <class T>
    class Twist : public Multivector<T, blades::e12, blades::e13, blades::e23, blades::e1i, blades::e2i, blades::e3i>
    {
      public:
        using Base = Multivector<T, blades::e12, blades::e13, blades::e23, blades::e1i, blades::e2i, blades::e3i>;
        using Parameters = typename Base::Parameters;

        using Angular = Multivector<T, blades::e12, blades::e13, blades::e23>;
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

        template <class E>
        Twist &operator=(const Expression<E, Base> &expression);

        Twist &operator+=(const Twist<T> &twist);

      protected:
      private:
    };

}  // namespace gafro

#include <gafro/physics/Twist.hxx>