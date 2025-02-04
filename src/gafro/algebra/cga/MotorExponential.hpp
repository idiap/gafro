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
                typename Rotor<T>::Generator b({ generator.template get<blades::e12>(),  //
                                                 generator.template get<blades::e13>(),  //
                                                 generator.template get<blades::e23>() });
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

        static Eigen::Matrix<T, 8, 6> getJacobian(const typename Motor<T>::Generator &b)
        {
            T b1 = b.template get<blades::e23>();
            T b2 = b.template get<blades::e13>();
            T b3 = b.template get<blades::e12>();
            T t1 = b.template get<blades::e1i>();
            T t2 = b.template get<blades::e2i>();
            T t3 = b.template get<blades::e3i>();

            typename Rotor<T>::Generator br({ b3, b2, b1 });

            T m1 = TypeTraits<T>::Value(0.0);
            T m2 = TypeTraits<T>::Value(0.0);
            T m3 = TypeTraits<T>::Value(0.0);
            T m4 = TypeTraits<T>::Value(0.0);

            T theta = br.norm();

            T m1b1 = TypeTraits<T>::Value(0.0);
            T m1b2 = TypeTraits<T>::Value(0.0);
            T m1b3 = TypeTraits<T>::Value(0.0);
            T m2b1 = TypeTraits<T>::Value(0.0);
            T m2b2 = TypeTraits<T>::Value(0.0);
            T m2b3 = TypeTraits<T>::Value(0.0);
            T m3b1 = TypeTraits<T>::Value(0.0);
            T m3b2 = TypeTraits<T>::Value(0.0);
            T m3b3 = TypeTraits<T>::Value(0.0);
            T m4b1 = TypeTraits<T>::Value(0.0);
            T m4b2 = TypeTraits<T>::Value(0.0);
            T m4b3 = TypeTraits<T>::Value(0.0);

            if (abs(theta) > 0)
            {
                br.normalize();

                Rotor<T> r(br, theta);

                m1 = r.template get<blades::scalar>();
                m2 = r.template get<blades::e23>();
                m3 = r.template get<blades::e13>();
                m4 = r.template get<blades::e12>();

                m1b1 = TypeTraits<T>::Value(-0.5) * b1 * sin(TypeTraits<T>::Value(0.5) * theta) / theta;
                m1b2 = TypeTraits<T>::Value(-0.5) * b2 * sin(TypeTraits<T>::Value(0.5) * theta) / theta;
                m1b3 = TypeTraits<T>::Value(-0.5) * b3 * sin(TypeTraits<T>::Value(0.5) * theta) / theta;

                m2b1 = sin(TypeTraits<T>::Value(0.5) * theta) * (b1 * b1 / pow(theta, 3) - TypeTraits<T>::Value(1.0) / theta) -
                       TypeTraits<T>::Value(0.5) * b1 * b1 * cos(TypeTraits<T>::Value(0.5) * theta) / (theta * theta);
                m2b2 = b1 * (b2 * sin(TypeTraits<T>::Value(0.5) * theta) / pow(theta, 3) -
                             TypeTraits<T>::Value(0.5) * b2 * cos(TypeTraits<T>::Value(0.5) * theta) / (theta * theta));
                m2b3 = b1 * (b3 * sin(TypeTraits<T>::Value(0.5) * theta) / pow(theta, 3) -
                             TypeTraits<T>::Value(0.5) * b3 * cos(TypeTraits<T>::Value(0.5) * theta) / (theta * theta));

                m3b1 = b2 * (b1 * sin(TypeTraits<T>::Value(0.5) * theta) / pow(theta, 3) -
                             TypeTraits<T>::Value(0.5) * b1 * cos(TypeTraits<T>::Value(0.5) * theta) / (theta * theta));
                m3b2 = sin(TypeTraits<T>::Value(0.5) * theta) * (b2 * b2 / pow(theta, 3) - TypeTraits<T>::Value(1.0) / theta) -
                       TypeTraits<T>::Value(0.5) * b2 * b2 * cos(TypeTraits<T>::Value(0.5) * theta) / (theta * theta);
                m3b3 = b2 * (b3 * sin(TypeTraits<T>::Value(0.5) * theta) / pow(theta, 3) -
                             TypeTraits<T>::Value(0.5) * b3 * cos(TypeTraits<T>::Value(0.5) * theta) / (theta * theta));

                m4b1 = b3 * (b1 * sin(TypeTraits<T>::Value(0.5) * theta) / pow(theta, 3) -
                             TypeTraits<T>::Value(0.5) * b1 * cos(TypeTraits<T>::Value(0.5) * theta) / (theta * theta));
                m4b2 = b3 * (b2 * sin(TypeTraits<T>::Value(0.5) * theta) / pow(theta, 3) -
                             TypeTraits<T>::Value(0.5) * b2 * cos(TypeTraits<T>::Value(0.5) * theta) / (theta * theta));
                m4b3 = sin(TypeTraits<T>::Value(0.5) * theta) * (b3 * b3 / pow(theta, 3) - TypeTraits<T>::Value(1.0) / theta) -
                       TypeTraits<T>::Value(0.5) * b3 * b3 * cos(TypeTraits<T>::Value(0.5) * theta) / (theta * theta);
            }
            else
            {
                m2 = TypeTraits<T>::Value(0.5) * b1;
                m3 = TypeTraits<T>::Value(0.5) * b2;
                m4 = TypeTraits<T>::Value(0.5) * b3;
            }

            T m5b1 = TypeTraits<T>::Value(-0.5) * (t1 * m1b1 - m3b1 * t3 - m4b1 * t2);
            T m5b2 = TypeTraits<T>::Value(-0.5) * (t1 * m1b2 - m3b2 * t3 - m4b2 * t2);
            T m5b3 = TypeTraits<T>::Value(-0.5) * (t1 * m1b3 - m3b3 * t3 - m4b3 * t2);

            T m6b1 = TypeTraits<T>::Value(-0.5) * (t2 * m1b1 - m2b1 * t3 + m4b1 * t1);
            T m6b2 = TypeTraits<T>::Value(-0.5) * (t2 * m1b2 - m2b2 * t3 + m4b2 * t1);
            T m6b3 = TypeTraits<T>::Value(-0.5) * (t2 * m1b3 - m2b3 * t3 + m4b3 * t1);

            T m7b1 = TypeTraits<T>::Value(-0.5) * (t3 * m1b1 + m2b1 * t2 + m3b1 * t1);
            T m7b2 = TypeTraits<T>::Value(-0.5) * (t3 * m1b2 + m2b2 * t2 + m3b2 * t1);
            T m7b3 = TypeTraits<T>::Value(-0.5) * (t3 * m1b3 + m2b3 * t2 + m3b3 * t1);

            T m8b1 = TypeTraits<T>::Value(-0.5) * (t1 * m2b1 - m3b1 * t2 + m4b1 * t3);
            T m8b2 = TypeTraits<T>::Value(-0.5) * (t1 * m2b2 - m3b2 * t2 + m4b2 * t3);
            T m8b3 = TypeTraits<T>::Value(-0.5) * (t1 * m2b3 - m3b3 * t2 + m4b3 * t3);

            T m5t1 = TypeTraits<T>::Value(-0.5) * m1;
            T m5t2 = TypeTraits<T>::Value(0.5) * m4;
            T m5t3 = TypeTraits<T>::Value(0.5) * m3;

            T m6t1 = TypeTraits<T>::Value(-0.5) * m4;
            T m6t2 = TypeTraits<T>::Value(-0.5) * m1;
            T m6t3 = TypeTraits<T>::Value(0.5) * m2;

            T m7t1 = TypeTraits<T>::Value(-0.5) * m3;
            T m7t2 = TypeTraits<T>::Value(-0.5) * m2;
            T m7t3 = TypeTraits<T>::Value(-0.5) * m1;

            T m8t1 = TypeTraits<T>::Value(-0.5) * m2;
            T m8t2 = TypeTraits<T>::Value(0.5) * m3;
            T m8t3 = TypeTraits<T>::Value(-0.5) * m4;

            Eigen::Matrix<T, 8, 6> exp_jacobian = Eigen::Matrix<T, 8, 6>::Zero();

            exp_jacobian.coeffRef(0, 0) = m1b3;
            exp_jacobian.coeffRef(1, 0) = m4b3;
            exp_jacobian.coeffRef(2, 0) = m3b3;
            exp_jacobian.coeffRef(3, 0) = m2b3;
            exp_jacobian.coeffRef(4, 0) = m5b3;
            exp_jacobian.coeffRef(5, 0) = m6b3;
            exp_jacobian.coeffRef(6, 0) = m7b3;
            exp_jacobian.coeffRef(7, 0) = m8b3;

            exp_jacobian.coeffRef(0, 1) = m1b2;
            exp_jacobian.coeffRef(1, 1) = m4b2;
            exp_jacobian.coeffRef(2, 1) = m3b2;
            exp_jacobian.coeffRef(3, 1) = m2b2;
            exp_jacobian.coeffRef(4, 1) = m5b2;
            exp_jacobian.coeffRef(5, 1) = m6b2;
            exp_jacobian.coeffRef(6, 1) = m7b2;
            exp_jacobian.coeffRef(7, 1) = m8b2;

            exp_jacobian.coeffRef(0, 2) = m1b1;
            exp_jacobian.coeffRef(1, 2) = m4b1;
            exp_jacobian.coeffRef(2, 2) = m3b1;
            exp_jacobian.coeffRef(3, 2) = m2b1;
            exp_jacobian.coeffRef(4, 2) = m5b1;
            exp_jacobian.coeffRef(5, 2) = m6b1;
            exp_jacobian.coeffRef(6, 2) = m7b1;
            exp_jacobian.coeffRef(7, 2) = m8b1;

            exp_jacobian.coeffRef(4, 3) = m5t1;
            exp_jacobian.coeffRef(5, 3) = m6t1;
            exp_jacobian.coeffRef(6, 3) = m7t1;
            exp_jacobian.coeffRef(7, 3) = m8t1;

            exp_jacobian.coeffRef(4, 4) = m5t2;
            exp_jacobian.coeffRef(5, 4) = m6t2;
            exp_jacobian.coeffRef(6, 4) = m7t2;
            exp_jacobian.coeffRef(7, 4) = m8t2;

            exp_jacobian.coeffRef(4, 5) = m5t3;
            exp_jacobian.coeffRef(5, 5) = m6t3;
            exp_jacobian.coeffRef(6, 5) = m7t3;
            exp_jacobian.coeffRef(7, 5) = m8t3;

            return exp_jacobian;
        }

      protected:
      private:
        Motor<T> result_;
    };

}  // namespace gafro