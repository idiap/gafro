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

#include <gafro/algebra/cga/ConformalTransformation.hpp>

namespace gafro
{

    template <class T>
    class ConformalTransformation<T>::Exponential
      : public UnaryExpression<ConformalTransformation<T>::Exponential, typename ConformalTransformation<T>::Generator, ConformalTransformation<T>>
    {
        Exponential(const typename ConformalTransformation<T>::Generator &generator);
        virtual ~Exponential() = default;

        template <int blade>
            requires(ConformalTransformation<T>::has(blade))  //
        T get() const;
    };

}  // namespace gafro