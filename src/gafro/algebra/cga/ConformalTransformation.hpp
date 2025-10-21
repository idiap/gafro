// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/Dilator.hxx>
#include <gafro/algebra/cga/Rotor.hxx>
#include <gafro/algebra/cga/Translator.hxx>
#include <gafro/algebra/cga/Transversion.hpp>
#include <gafro/algebra/cga/Versor.hpp>

namespace gafro
{

    template <class T>
    class ConformalTransformation
      : public Versor<ConformalTransformation<T>, T, blades::scalar, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23,
                      blades::e0123, blades::e0i, blades::e1i, blades::e2i, blades::e012i, blades::e3i, blades::e013i, blades::e023i, blades::e123i>
    {
      public:
        using Base =
          Versor<ConformalTransformation<T>, T, blades::scalar, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23,
                 blades::e0123, blades::e0i, blades::e1i, blades::e2i, blades::e012i, blades::e3i, blades::e013i, blades::e023i, blades::e123i>;

        using Parameters = typename Base::Parameters;

        using Generator = Multivector<T, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23, blades::e0i, blades::e1i,
                                      blades::e2i, blades::e3i>;

        using Base::Base;

        class CanonicalDecomposition
        {
          public:
            CanonicalDecomposition(const ConformalTransformation &transformation);

            const Dilator<T> &getDilator() const;

            const Rotor<T> &getRotor() const;

            const Transversion<T> &getTransversion() const;

            const Translator<T> &getTranslator() const;

          private:
            Dilator<T> dilator_;

            Rotor<T> rotor_;

            Transversion<T> transversion_;

            Translator<T> translator_;
        };

        class Exponential;
        class Logarithm;

        ConformalTransformation();

        ConformalTransformation(const Generator &generator);

        virtual ~ConformalTransformation();

        Generator log() const;

        CanonicalDecomposition getCanonicalDecomposition() const;

      protected:
      private:
      public:
        static ConformalTransformation exp(const typename Generator::Parameters &generator);

        static ConformalTransformation exp(const Generator &generator);
    };

}  // namespace gafro

#include <gafro/algebra/cga/ConformalTransformation.hxx>