// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

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

            T factor1 = TypeTraits<T>::Value(0.0);
            T factor2 = TypeTraits<T>::Value(0.0);

            if (m1 != TypeTraits<T>::Value(1.0))
            {
                factor1 = TypeTraits<T>::Value(-2.0) * (TypeTraits<T>::Value(1.0) / (m1 * m1 - TypeTraits<T>::Value(1.0)) +
                                                        m1 * acos(m1) / pow(TypeTraits<T>::Value(1.0) - m1 * m1, 1.5));
                factor2 = -(TypeTraits<T>::Value(2.0) * acos(m1) / sin(acos(m1)));
            }

            jacobian.coeffRef(0, 0) = factor1 * m4;
            jacobian.coeffRef(0, 1) = factor2;

            jacobian.coeffRef(1, 0) = factor1 * m3;
            jacobian.coeffRef(1, 2) = factor2;

            jacobian.coeffRef(2, 0) = factor1 * m2;
            jacobian.coeffRef(2, 3) = factor2;

            jacobian.coeffRef(3, 0) = TypeTraits<T>::Value(-2.0) * m5;
            jacobian.coeffRef(3, 1) = TypeTraits<T>::Value(-2.0) * m6;
            jacobian.coeffRef(3, 2) = TypeTraits<T>::Value(-2.0) * m7;
            jacobian.coeffRef(3, 3) = TypeTraits<T>::Value(-2.0) * m8;
            jacobian.coeffRef(3, 4) = TypeTraits<T>::Value(-2.0) * m1;
            jacobian.coeffRef(3, 5) = TypeTraits<T>::Value(-2.0) * m4;
            jacobian.coeffRef(3, 6) = TypeTraits<T>::Value(-2.0) * m3;
            jacobian.coeffRef(3, 7) = TypeTraits<T>::Value(-2.0) * m2;

            jacobian.coeffRef(4, 0) = TypeTraits<T>::Value(-2.0) * m6;
            jacobian.coeffRef(4, 1) = TypeTraits<T>::Value(2.0) * m5;
            jacobian.coeffRef(4, 2) = TypeTraits<T>::Value(2.0) * m8;
            jacobian.coeffRef(4, 3) = TypeTraits<T>::Value(-2.0) * m7;
            jacobian.coeffRef(4, 4) = TypeTraits<T>::Value(2.0) * m4;
            jacobian.coeffRef(4, 5) = TypeTraits<T>::Value(-2.0) * m1;
            jacobian.coeffRef(4, 6) = TypeTraits<T>::Value(-2.0) * m2;
            jacobian.coeffRef(4, 7) = TypeTraits<T>::Value(2.0) * m3;

            jacobian.coeffRef(5, 0) = TypeTraits<T>::Value(-2.0) * m7;
            jacobian.coeffRef(5, 1) = TypeTraits<T>::Value(-2.0) * m8;
            jacobian.coeffRef(5, 2) = TypeTraits<T>::Value(2.0) * m5;
            jacobian.coeffRef(5, 3) = TypeTraits<T>::Value(2.0) * m6;
            jacobian.coeffRef(5, 4) = TypeTraits<T>::Value(2.0) * m3;
            jacobian.coeffRef(5, 5) = TypeTraits<T>::Value(2.0) * m2;
            jacobian.coeffRef(5, 6) = TypeTraits<T>::Value(-2.0) * m1;
            jacobian.coeffRef(5, 7) = TypeTraits<T>::Value(-2.0) * m4;

            return jacobian;
        }

      protected:
      private:
    };

}  // namespace gafro