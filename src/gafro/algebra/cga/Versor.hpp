// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/cga/Blades.hpp>

namespace gafro
{
    template <class Object, class Versor>
    class SandwichProduct;

    template <class Derived, class T, int... index>
    class Versor : public Multivector<T, index...>
    {
      public:
        using Base = Multivector<T, index...>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        Versor();

        Versor(const Base &base);

        Versor(Base &&base);

        virtual ~Versor();

        template <class Object>
        SandwichProduct<Object, Derived> apply(const Object &object) const;

      protected:
      private:
      public:
    };

}  // namespace gafro