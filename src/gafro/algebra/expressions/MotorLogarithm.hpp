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
    class Motor<T>::Logarithm : public UnaryExpression<Motor<T>::Logarithm, Motor<T>, typename Motor<T>::Generator>
    {
      public:
        Logarithm(const Motor<T> &motor) : UnaryExpression<Motor<T>::Logarithm, Motor<T>, typename Motor<T>::Generator>(motor) {}

        virtual ~Logarithm() = default;

        template <int blade>
            requires(Motor<T>::has(blade))  //
        T get() const
        {
            const Motor<T> &motor = this->operand();

            switch (blade)
            {
            case blades::e23:
                if (TypeTraits<T>::greaterEqual(abs(motor.template get<blades::scalar>()), TypeTraits<T>::Value(1.0 - 1e-10)))
                {
                    return TypeTraits<T>::Zero();
                }

                return motor.template get<blades::e23>() * TypeTraits<T>::Value(-2.0) * acos(motor.template get<blades::scalar>()) /
                       (sin(acos(motor.template get<blades::scalar>())) + TypeTraits<T>::Value(1e-10));
            case blades::e13:
                if (TypeTraits<T>::greaterEqual(abs(motor.template get<blades::scalar>()), TypeTraits<T>::Value(1.0 - 1e-10)))
                {
                    return TypeTraits<T>::Zero();
                }

                return motor.template get<blades::e13>() * TypeTraits<T>::Value(-2.0) * acos(motor.template get<blades::scalar>()) /
                       (sin(acos(motor.template get<blades::scalar>())) + TypeTraits<T>::Value(1e-10));
            case blades::e12:
                if (TypeTraits<T>::greaterEqual(abs(motor.template get<blades::scalar>()), TypeTraits<T>::Value(1.0 - 1e-10)))
                {
                    return TypeTraits<T>::Zero();
                }

                return motor.template get<blades::e12>() * TypeTraits<T>::Value(-2.0) * acos(motor.template get<blades::scalar>()) /
                       (sin(acos(motor.template get<blades::scalar>())) + TypeTraits<T>::Value(1e-10));
            case blades::e1i:
                return TypeTraits<T>::Value(-2.0) * (motor.template get<blades::scalar>() * motor.template get<blades::e1i>() +
                                                     motor.template get<blades::e12>() * motor.template get<blades::e2i>() +
                                                     motor.template get<blades::e13>() * motor.template get<blades::e3i>() +
                                                     motor.template get<blades::e23>() * motor.template get<blades::e123i>());
            case blades::e2i:
                return TypeTraits<T>::Value(-2.0) * (-motor.template get<blades::e12>() * motor.template get<blades::e1i>() +
                                                     motor.template get<blades::scalar>() * motor.template get<blades::e2i>() +
                                                     motor.template get<blades::e23>() * motor.template get<blades::e3i>() -
                                                     motor.template get<blades::e13>() * motor.template get<blades::e123i>());
            case blades::e3i:
                return TypeTraits<T>::Value(-2.0) * (-motor.template get<blades::e13>() * motor.template get<blades::e1i>() -
                                                     motor.template get<blades::e23>() * motor.template get<blades::e2i>() +
                                                     motor.template get<blades::scalar>() * motor.template get<blades::e3i>() +
                                                     motor.template get<blades::e12>() * motor.template get<blades::e123i>());
            default:
                return TypeTraits<T>::Zero();
            }
        }

      protected:
      private:
    };

}  // namespace gafro