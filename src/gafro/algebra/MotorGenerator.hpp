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
#include <gafro/algebra/expressions/Expression.hpp>

namespace gafro
{

    template <class T>
    class Motor<T>::Generator : public Multivector<T, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i>
    {
      public:
        using Base = Multivector<T, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i>;

        using Parameters = typename Base::Parameters;

        Generator();

        Generator(const Base &other);

        Generator(const Parameters &parameters);

        Generator(const Eigen::Matrix<T, 3, 1> &p1, const Eigen::Matrix<T, 3, 1> &p2);

        template <class E>
        Generator(const Expression<E, Generator> &expression);

        template <class E, class R>
        requires(std::is_convertible<R, Generator>::value)  //
          Generator(const Expression<E, Generator> &expression)
        {
            this->template set<blades::e23>(expression.template get<blades::e23>());
            this->template set<blades::e13>(expression.template get<blades::e13>());
            this->template set<blades::e12>(expression.template get<blades::e12>());
            this->template set<blades::e1i>(expression.template get<blades::e1i>());
            this->template set<blades::e2i>(expression.template get<blades::e2i>());
            this->template set<blades::e3i>(expression.template get<blades::e3i>());
        }

        virtual ~Generator() = default;

        template <class E, class R>
        // requires(std::is_convertible<typename R::Type, Generator>::value)  //
        Generator &operator=(const Expression<E, R> &expression)
        {
            this->template set<blades::e23>(expression.template get<blades::e23>());
            this->template set<blades::e13>(expression.template get<blades::e13>());
            this->template set<blades::e12>(expression.template get<blades::e12>());
            this->template set<blades::e1i>(expression.template get<blades::e1i>());
            this->template set<blades::e2i>(expression.template get<blades::e2i>());
            this->template set<blades::e3i>(expression.template get<blades::e3i>());

            return *this;
        }

      protected:
      private:
    };

}  // namespace gafro