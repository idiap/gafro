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

#include <gafro/algebra/Motor.hpp>
#include <gafro/algebra/RotorGenerator.hpp>
#include <gafro/algebra/expressions/Expression.hpp>

namespace gafro
{

    template <class T>
    class Motor<T>::Generator : public Multivector<T, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i>
    {
      public:
        using Base = Multivector<T, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i>;

        using Parameters = typename Base::Parameters;

        //

        using Base::Base;
        using Base::operator=;

        //

        Generator();

        Generator(const Base &other);

        Generator(const Parameters &parameters);

        Generator(const Eigen::Matrix<T, 3, 1> &p1, const Eigen::Matrix<T, 3, 1> &p2);

        virtual ~Generator() = default;

        typename Rotor<T>::Generator getRotorGenerator() const;

        typename Translator<T>::Generator getTranslatorGenerator() const;

      protected:
      private:
    };

    template <class T>
    using MotorGenerator = typename Motor<T>::Generator;

}  // namespace gafro