// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/Rotor.hpp>

namespace gafro
{
    template <typename T>
    class Rotor<T>::Generator : public Multivector<T, blades::e12, blades::e13, blades::e23>
    {
      public:
        using Base = Multivector<T, blades::e12, blades::e13, blades::e23>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        Generator() = default;

        Generator(const Base &other);

        Generator(const Parameters &parameters);

        template <class E>
        Generator(const Expression<E, Base> &expression);

        virtual ~Generator() = default;

        const T &e23() const;

        const T &e13() const;

        const T &e12() const;

        template <class E>
        Generator &operator=(const Expression<E, Base> &expression);

      protected:
      private:
    };

}  // namespace gafro