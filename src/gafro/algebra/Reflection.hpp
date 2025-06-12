// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/GeometricProduct.hpp>
#include <gafro/algebra/UnaryExpression.hpp>

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