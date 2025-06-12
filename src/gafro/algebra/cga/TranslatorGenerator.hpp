// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Translator.hpp>

namespace gafro
{
    template <class T>
    class Translator<T>::Generator : public Multivector<T, blades::e1i, blades::e2i, blades::e3i>
    {
      public:
        using Base = Multivector<T, blades::e1i, blades::e2i, blades::e3i>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        Generator() = default;

        Generator(const Base &other);

        Generator(const Parameters &parameters);

        const T &x() const;

        const T &y() const;

        const T &z() const;

      private:
    };

}  // namespace gafro