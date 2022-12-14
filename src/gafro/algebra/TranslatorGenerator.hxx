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

#include <gafro/algebra/TranslatorGenerator.hpp>

namespace gafro
{

    template <class T>
    Translator<T>::Generator::Generator(const Base &other) : Base(other)
    {}

    template <class T>
    Translator<T>::Generator::Generator(const Generator &other) : Base(other)
    {}

    template <class T>
    Translator<T>::Generator::Generator(const Parameters &parameters) : Base(parameters)
    {}

    template <class T>
    const T &Translator<T>::Generator::x() const
    {
        return this->vector().coeffRef(0, 0);
    }

    template <class T>
    const T &Translator<T>::Generator::y() const
    {
        return this->vector().coeffRef(1, 0);
    }

    template <class T>
    const T &Translator<T>::Generator::z() const
    {
        return this->vector().coeffRef(2, 0);
    }

}  // namespace gafro