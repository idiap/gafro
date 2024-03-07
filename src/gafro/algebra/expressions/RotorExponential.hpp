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

#include <gafro/algebra/Rotor.hpp>
#include <gafro/algebra/expressions/UnaryExpression.hpp>

namespace gafro
{

    template <class T>
    class Rotor<T>::Exponential : public UnaryExpression<Rotor<T>::Exponential, typename Rotor<T>::Generator, Rotor<T>>
    {
      public:
        Exponential(const typename Rotor<T>::Generator &generator)
          : UnaryExpression<Rotor<T>::Exponential, typename Rotor<T>::Generator, Rotor<T>>(generator)
        {
            if (generator.template get<blades::e23>() * generator.template get<blades::e23>()      //
                  + generator.template get<blades::e13>() * generator.template get<blades::e13>()  //
                  + generator.template get<blades::e12>() * generator.template get<blades::e12>() >
                T(0.0))
            {
                typename Rotor<T>::Generator b = generator;

                T theta = b.norm();

                b.normalize();

                result_ = Rotor<T>(b, theta);
            }
        }

        virtual ~Exponential() = default;

        template <int blade>
            requires(Rotor<T>::has(blade))  //
        T get() const
        {
            switch (blade)
            {
            case blades::scalar:
                return result_.template get<blades::scalar>();
            case blades::e23:
                return result_.template get<blades::e23>();
            case blades::e13:
                return result_.template get<blades::e13>();
            case blades::e12:
                return result_.template get<blades::e12>();
            default:
                return 1.0;
            }
        }

      protected:
      private:
        Rotor<T> result_;
    };

}  // namespace gafro