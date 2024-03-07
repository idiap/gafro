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
#include <gafro/algebra/expressions/GeometricProduct.hpp>
#include <gafro/algebra/expressions/UnaryExpression.hpp>

namespace gafro
{

    template <class Object, class Mirror>
    class Reflection : public Expression<Reflection<Object, Mirror>, Object>
    {
      public:
        using Type = Object;
        using Vtype = typename Object::Vtype;
        using Product = decltype(Mirror() * Object() * Mirror());

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;

        Reflection(const Object &object, const Mirror &versor) : product_(versor * object * versor) {}

        virtual ~Reflection() = default;

        template <int blade>
            requires(Type::has(blade))  //
        Vtype get() const
        {
            return product_.template get<blade>();
        }

      protected:
      private:
        Product product_;
    };

}  // namespace gafro