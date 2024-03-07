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
#include <gafro/algebra/Rotor.hpp>

namespace gafro
{
    template <typename T>
    class Rotor<T>::Generator : public Multivector<T, blades::e23, blades::e13, blades::e12>
    {
      public:
        using Base = Multivector<T, blades::e23, blades::e13, blades::e12>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        Generator() = default;

        Generator(const Generator &other);

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