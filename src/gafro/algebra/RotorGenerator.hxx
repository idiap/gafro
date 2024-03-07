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

#include <gafro/algebra/RotorGenerator.hpp>

namespace gafro
{

    template <class T>
    Rotor<T>::Generator::Generator(const Base &other) : Base(other)
    {}

    template <class T>
    Rotor<T>::Generator::Generator(const Generator &other) : Base(other)
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