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

#include <gafro/algebra/Blade.hpp>
#include <gafro/algebra/Multivector.hxx>

template <class T, class V>
std::ostream &operator<<(std::ostream &ostream, const gafro::Blade<T, V> &blade)
{
    using namespace ga3c;

    ostream << blade.value();

    return ostream;
}

// template <class T>
// std::ostream &operator<<(std::ostream &ostream, const gafro::Multivector<T> &mv)
// {
//     using namespace ga3c;

//     ostream << mv.value();

//     return ostream;
// }

// template <class T, class T1, class T2>
// auto operator+(const gafro::Blade<T, T1> &b1, const gafro::Blade<T, T2> &b2)
// {
//     return gafro::Multivector<decltype(b1.value() + b2.value())>(b1.value() + b2.value());
// }

// template <class T, class T1, class T2>
// auto operator+(const gafro::Blade<T, T1> &b1, const gafro::Multivector<T2> &b2)
// {
//     return gafro::Multivector<decltype(b1.value() + b2.value())>(b1.value() + b2.value());
// }

// template <class T, class T1, class T2>
// auto operator+(const gafro::Multivector<T1> &b1, const gafro::Multivector<T2> &b2)
// {
//     return gafro::Multivector<decltype(b1.value() + b2.value())>(b1.value() + b2.value());
// }

// template <class T, class T1, class T2>
// auto operator+(const gafro::Multivector<T1> &b1, const gafro::Blade<T, T2> &b2)
// {
//     return gafro::Multivector<decltype(b1.value() + b2.value())>(b1.value() + b2.value());
// }