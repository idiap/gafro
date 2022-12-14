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
    class Motor<T>::Exponential : public UnaryExpression<Motor<T>::Exponential, typename Motor<T>::Generator, Motor<T>>
    {
      public:
        Exponential(const typename Motor<T>::Generator &generator)
          : UnaryExpression<Motor<T>::Exponential, typename Motor<T>::Generator, Motor<T>>(generator)
        {
            if (generator.template get<blades::e23>() * generator.template get<blades::e23>()      //
                  + generator.template get<blades::e13>() * generator.template get<blades::e13>()  //
                  + generator.template get<blades::e12>() * generator.template get<blades::e12>() >
                T(0.0))
            {
                typename Rotor<T>::Generator b({ generator.template get<blades::e23>(),  //
                                                 generator.template get<blades::e13>(),  //
                                                 generator.template get<blades::e12>() },
                                               false);
                T theta = b.norm();

                typename Translator<T>::Generator t({ generator.template get<blades::e1i>(),  //
                                                      generator.template get<blades::e2i>(),  //
                                                      generator.template get<blades::e3i>() });

                b.normalize();

                result_ = Motor<T>(Translator<T>(t), Rotor<T>(b, theta));
            }
        }

        virtual ~Exponential() = default;

        template <int blade>
        requires(Motor<T>::has(blade))  //
          T get()
        const
        {
            const Motor<T>::Generator &generator = this->operand();

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
            case blades::e1i:
                return result_.template get<blades::e1i>();
            case blades::e2i:
                return result_.template get<blades::e2i>();
            case blades::e3i:
                return result_.template get<blades::e3i>();
            case blades::e123i:
                return result_.template get<blades::e123i>();
            default:
                return 1.0;
            }
        }

      protected:
      private:
        Motor<T> result_;
    };

}  // namespace gafro