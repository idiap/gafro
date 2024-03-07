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

#include <gafro/algebra/Blades.hpp>
#include <gafro/algebra/Versor.hpp>

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
        static Translator exp(const Generator &generator);
    };

}  // namespace gafro