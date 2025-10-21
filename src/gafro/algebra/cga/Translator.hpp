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
    class Translator : public Versor<Translator<T>, T, blades::scalar, blades::e1i, blades::e2i, blades::e3i>
    {
      public:
        using Base = Versor<Translator<T>, T, blades::scalar, blades::e1i, blades::e2i, blades::e3i>;

        using Parameters = typename Base::Parameters;

        class Generator;

        using Base::Base;

        Translator();

        Translator(const Generator &generator);

        virtual ~Translator();

        Generator log() const;

        Eigen::Vector<T, 3> toTranslationVector() const;

        Eigen::Matrix<T, 3, 3> toSkewSymmetricMatrix() const;

      protected:
      private:
      public:
        static Translator exp(const T &e1i, const T &e2i, const T &e3i);

        static Translator exp(const Eigen::Vector<T, 3> &generator);

        static Translator exp(const Generator &generator);

        static Translator X(const T &e1i);

        static Translator Y(const T &e2i);

        static Translator Z(const T &e3i);
    };

}  // namespace gafro