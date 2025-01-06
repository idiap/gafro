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

#include <gafro/algebra/UnaryExpression.hpp>
#include <gafro/algebra/cga/Motor.hpp>

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
                if (TypeTraits<T>::greaterEqual(fabs(motor.template get<blades::scalar>()), TypeTraits<T>::Value(1.0 - 1e-10)))
                {
                    return TypeTraits<T>::Zero();
                }

                return motor.template get<blades::e23>() * TypeTraits<T>::Value(-2.0) * acos(motor.template get<blades::scalar>()) /
                       (sin(acos(motor.template get<blades::scalar>())) + TypeTraits<T>::Value(1e-10));
            case blades::e13:
                if (TypeTraits<T>::greaterEqual(fabs(motor.template get<blades::scalar>()), TypeTraits<T>::Value(1.0 - 1e-10)))
                {
                    return TypeTraits<T>::Zero();
                }

                return motor.template get<blades::e13>() * TypeTraits<T>::Value(-2.0) * acos(motor.template get<blades::scalar>()) /
                       (sin(acos(motor.template get<blades::scalar>())) + TypeTraits<T>::Value(1e-10));
            case blades::e12:
                if (TypeTraits<T>::greaterEqual(fabs(motor.template get<blades::scalar>()), TypeTraits<T>::Value(1.0 - 1e-10)))
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

        static Eigen::Matrix<T, 6, 8> getJacobian(const Motor<T> &motor)
        {
            T m1 = motor.vector().coeff(0, 0);
            T m2 = motor.vector().coeff(3, 0);
            T m3 = motor.vector().coeff(2, 0);
            T m4 = motor.vector().coeff(1, 0);
            T m5 = motor.vector().coeff(4, 0);
            T m6 = motor.vector().coeff(5, 0);
            T m7 = motor.vector().coeff(6, 0);
            T m8 = motor.vector().coeff(7, 0);

            Eigen::Matrix<T, 6, 8> jacobian = Eigen::Matrix<T, 6, 8>::Zero();

            T factor1 = 0.0;
            T factor2 = 0.0;

            if (m1 != 1.0)
            {
                factor1 = -2.0 * (1.0 / (m1 * m1 - 1.0) + m1 * acos(m1) / pow(1.0 - m1 * m1, 1.5));
                factor2 = -(2.0 * acos(m1) / sin(acos(m1)));
            }

            jacobian.coeffRef(0, 0) = factor1 * m4;
            jacobian.coeffRef(0, 1) = factor2;

            jacobian.coeffRef(1, 0) = factor1 * m3;
            jacobian.coeffRef(1, 2) = factor2;

            jacobian.coeffRef(2, 0) = factor1 * m2;
            jacobian.coeffRef(2, 3) = factor2;

            jacobian.coeffRef(3, 0) = -2.0 * m5;
            jacobian.coeffRef(3, 1) = -2.0 * m6;
            jacobian.coeffRef(3, 2) = -2.0 * m7;
            jacobian.coeffRef(3, 3) = -2.0 * m8;
            jacobian.coeffRef(3, 4) = -2.0 * m1;
            jacobian.coeffRef(3, 5) = -2.0 * m4;
            jacobian.coeffRef(3, 6) = -2.0 * m3;
            jacobian.coeffRef(3, 7) = -2.0 * m2;

            jacobian.coeffRef(4, 0) = -2.0 * m6;
            jacobian.coeffRef(4, 1) = 2.0 * m5;
            jacobian.coeffRef(4, 2) = 2.0 * m8;
            jacobian.coeffRef(4, 3) = -2.0 * m7;
            jacobian.coeffRef(4, 4) = 2.0 * m4;
            jacobian.coeffRef(4, 5) = -2.0 * m1;
            jacobian.coeffRef(4, 6) = -2.0 * m2;
            jacobian.coeffRef(4, 7) = 2.0 * m3;

            jacobian.coeffRef(5, 0) = -2.0 * m7;
            jacobian.coeffRef(5, 1) = -2.0 * m8;
            jacobian.coeffRef(5, 2) = 2.0 * m5;
            jacobian.coeffRef(5, 3) = 2.0 * m6;
            jacobian.coeffRef(5, 4) = 2.0 * m3;
            jacobian.coeffRef(5, 5) = 2.0 * m2;
            jacobian.coeffRef(5, 6) = -2.0 * m1;
            jacobian.coeffRef(5, 7) = -2.0 * m4;

            return jacobian;
        }

      protected:
      private:
    };

}  // namespace gafro