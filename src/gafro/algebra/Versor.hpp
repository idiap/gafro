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
#include <gafro/algebra/Multivector.hxx>

namespace gafro
{
    template <class Object, class Versor>
    class SandwichProduct;

    template <class Derived, class T, int... index>
    class Versor : public Multivector<T, index...>
    {
      public:
        using Base = Multivector<T, index...>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        Versor();

        Versor(const Base &base);

        Versor(Base &&base);

        virtual ~Versor();

        template <class Object>
        SandwichProduct<Object, Derived> apply(const Object &object) const;

      protected:
      private:
      public:
    };

}  // namespace gafro