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
    class TangentVector : public Multivector<T, blades::e01, blades::e02, blades::e03>
    {
      public:
        using Base = Multivector<T, blades::e01, blades::e02, blades::e03>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        TangentVector();

        TangentVector(const Base &other);

        TangentVector(const T &x, const T &y, const T &z);

        virtual ~TangentVector() = default;

      protected:
      private:
      public:
    };

}  // namespace gafro