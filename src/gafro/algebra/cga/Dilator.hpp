// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/Versor.hpp>

namespace gafro
{
    template <class T>
    class Dilator : public Versor<Dilator<T>, T, blades::scalar, blades::e0i>
    {
      public:
        using Base = Versor<Dilator<T>, T, blades::scalar, blades::e0i>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        using Generator = Multivector<T, blades::e0i>;

        Dilator();

        Dilator(const T &dilation);

        virtual ~Dilator();

        Generator logarithm() const;

      protected:
      private:
      public:
        static Dilator exp(const Generator &generator);
    };

}  // namespace gafro

#include <gafro/algebra/cga/Dilator.hxx>