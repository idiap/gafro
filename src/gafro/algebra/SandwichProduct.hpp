// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Expression.hpp>
#include <gafro/algebra/GeometricProduct.hpp>

namespace gafro
{

    template <class Object, class Versor>
    class SandwichProduct : public Expression<SandwichProduct<Object, Versor>, Object>
    {
      public:
        using Type = Object;
        using Vtype = typename Object::Vtype;
        using Product = decltype(Versor() * Object() * Versor().reverse());

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;

        SandwichProduct(const Object &object, const Versor &versor)  //
          : product_(versor * object * versor.reverse())
        {}

        virtual ~SandwichProduct() = default;

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