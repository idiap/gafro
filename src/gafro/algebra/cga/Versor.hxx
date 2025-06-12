// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/SandwichProduct.hpp>
#include <gafro/algebra/cga/Versor.hpp>

namespace gafro
{

    template <class Derived, class T, int... index>
    Versor<Derived, T, index...>::Versor() = default;

    template <class Derived, class T, int... index>
    Versor<Derived, T, index...>::Versor(const Base &base) : Base(base)
    {}

    template <class Derived, class T, int... index>
    Versor<Derived, T, index...>::Versor(Base &&base) : Base(std::move(base))
    {}

    template <class Derived, class T, int... index>
    Versor<Derived, T, index...>::~Versor() = default;

    template <class Derived, class T, int... index>
    template <class Object>
    SandwichProduct<Object, Derived> Versor<Derived, T, index...>::apply(const Object &object) const
    {
        return SandwichProduct(*static_cast<const Object *>(&object), *static_cast<const Derived *>(this));
    }

}  // namespace gafro