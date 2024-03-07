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
#include <gafro/algebra/expressions/UnaryExpression.hpp>

namespace gafro
{

    template <class T>
    class Motor<T>::Exponential : public UnaryExpression<Motor<T>::Exponential, typename Motor<T>::Generator, Motor<T>>
    {
      public:
        Exponential(const typename Motor<T>::Generator &generator)
          : UnaryExpression<Motor<T>::Exponential, typename Motor<T>::Generator, Motor<T>>(generator)
        {
            if (TypeTraits<T>::greater(generator.template get<blades::e23>() * generator.template get<blades::e23>()       //
                                         + generator.template get<blades::e13>() * generator.template get<blades::e13>()   //
                                         + generator.template get<blades::e12>() * generator.template get<blades::e12>(),  //
                                       TypeTraits<T>::Zero()))
            {
                typename Rotor<T>::Generator b({ generator.template get<blades::e23>(),  //
                                                 generator.template get<blades::e13>(),  //
                                                 generator.template get<blades::e12>() });
                T theta = b.norm();

                typename Translator<T>::Generator t({ generator.template get<blades::e1i>(),  //
                                                      generator.template get<blades::e2i>(),  //
                                                      generator.template get<blades::e3i>() });

                b.normalize();

                result_ = Motor<T>(Translator<T>(t), Rotor<T>(b, theta));
            }
            else
            {
                typename Translator<T>::Generator t({ generator.template get<blades::e1i>(),  //
                                                      generator.template get<blades::e2i>(),  //
                                                      generator.template get<blades::e3i>() });

                result_ = Motor<T>(Translator<T>(t), Rotor<T>());
            }
        }

        virtual ~Exponential() = default;

        template <int blade>
            requires(Motor<T>::has(blade))  //
        T get() const
        {
            return result_.template get<blade>();
        }

      protected:
      private:
        Motor<T> result_;
    };

}  // namespace gafro