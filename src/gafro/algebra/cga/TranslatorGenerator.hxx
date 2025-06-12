// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/TranslatorGenerator.hpp>

namespace gafro
{

    template <class T>
    Translator<T>::Generator::Generator(const Base &other) : Base(other)
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