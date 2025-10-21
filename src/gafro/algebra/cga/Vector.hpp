// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/Rotor.hpp>

namespace gafro
{
    template <class T>
    class Vector : public Multivector<T, blades::e1, blades::e2, blades::e3>
    {
      public:
        using Base = Multivector<T, blades::e1, blades::e2, blades::e3>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        Vector();

        Vector(const Base &other);

        Vector(const T &x, const T &y, const T &z);

        virtual ~Vector() = default;

        Rotor<T> getRotor(const Vector &other) const;

      public:
        static Vector Z(const T &z);

        static Vector UnitZ();
    };

}  // namespace gafro

#include <gafro/algebra/cga/Vector.hxx>