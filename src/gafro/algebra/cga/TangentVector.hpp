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