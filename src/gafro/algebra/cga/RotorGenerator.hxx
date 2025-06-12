// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/RotorGenerator.hpp>

namespace gafro
{

    template <class T>
    Rotor<T>::Generator::Generator(const Base &other) : Base(other)
    {}

    template <class T>
    Rotor<T>::Generator::Generator(const Parameters &parameters) : Base(parameters)
    {}

    template <class T>
    template <class E>
    Rotor<T>::Generator::Generator(const Expression<E, Base> &expression)
    {
        this->template set<blades::e23>(expression.template get<blades::e23>());
        this->template set<blades::e13>(expression.template get<blades::e13>());
        this->template set<blades::e12>(expression.template get<blades::e12>());
    }

    template <class T>
    const T &Rotor<T>::Generator::e23() const
    {
        return this->template get<blades::e23>();
    }

    template <class T>
    const T &Rotor<T>::Generator::e13() const
    {
        return this->template get<blades::e13>();
    }

    template <class T>
    const T &Rotor<T>::Generator::e12() const
    {
        return this->template get<blades::e12>();
    }

    template <class T>
    template <class E>
    typename Rotor<T>::Generator &Rotor<T>::Generator::operator=(const Expression<E, Base> &expression)
    {
        this->template set<blades::e23>(expression.template get<blades::e23>());
        this->template set<blades::e13>(expression.template get<blades::e13>());
        this->template set<blades::e12>(expression.template get<blades::e12>());

        return *this;
    }

}  // namespace gafro