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
    template <class T>
    class DirectionVector : public Multivector<T, blades::e1i, blades::e2i, blades::e3i>
    {
      public:
        using Base = Multivector<T, blades::e1i, blades::e2i, blades::e3i>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        DirectionVector();

        DirectionVector(const Base &other);

        DirectionVector(const T &x, const T &y, const T &z);

        virtual ~DirectionVector() = default;

      protected:
      private:
      public:
    };

}  // namespace gafro