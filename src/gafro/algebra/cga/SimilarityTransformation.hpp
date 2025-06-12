// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/Dilator.hxx>
#include <gafro/algebra/cga/Motor.hxx>
#include <gafro/algebra/cga/Rotor.hxx>
#include <gafro/algebra/cga/Translator.hxx>
#include <gafro/algebra/cga/Transversion.hpp>
#include <gafro/algebra/cga/Versor.hpp>

namespace gafro
{

    template <class T>
    class SimilarityTransformation : public Versor<SimilarityTransformation<T>, T, blades::scalar, blades::e12, blades::e13, blades::e23, blades::e0i,
                                                   blades::e1i, blades::e2i, blades::e012i, blades::e3i, blades::e013i, blades::e023i, blades::e123i>
    {
      public:
        using Base = Versor<SimilarityTransformation<T>, T, blades::scalar, blades::e12, blades::e13, blades::e23, blades::e0i, blades::e1i,
                            blades::e2i, blades::e012i, blades::e3i, blades::e013i, blades::e023i, blades::e123i>;

        using Parameters = typename Base::Parameters;

        using Generator = Multivector<T, blades::e12, blades::e13, blades::e23, blades::e0i, blades::e1i, blades::e2i, blades::e3i>;

        using Base::Base;

        class CanonicalDecomposition
        {
          public:
            CanonicalDecomposition(const SimilarityTransformation &transformation);

            const Translator<T> &getTranslator() const;

            const Rotor<T> &getRotor() const;

            const Dilator<T> &getDilator() const;

          private:
            Translator<T> translator_;

            Rotor<T> rotor_;

            Dilator<T> dilator_;
        };

        class Exponential;
        class Logarithm;

        SimilarityTransformation();

        SimilarityTransformation(const Generator &generator);

        virtual ~SimilarityTransformation();

        Generator log() const;

        CanonicalDecomposition getCanonicalDecomposition() const;

      protected:
      private:
      public:
        static SimilarityTransformation exp(const typename Generator::Parameters &generator);

        static SimilarityTransformation exp(const Generator &generator);
    };

}  // namespace gafro

#include <gafro/algebra/cga/SimilarityTransformation.hxx>