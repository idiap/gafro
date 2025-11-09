// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/dqa/DualQuaternionAlgebra.hpp>

namespace gafro
{
    template <class T>
    class Motor;
}

namespace gafro::dqa
{

    template <class T>
    class Point;

    template <class T>
    class Line;

    template <class T>
    class Plane;

    template <class T>
    class Quaternion : public Multivector<T, idx::scalar, idx::i, idx::j, idx::k>
    {
      public:
        using Base = Multivector<T, idx::scalar, idx::i, idx::j, idx::k>;
    };

    template <class T>
    class DualQuaternion : public Multivector<T, 0b000, 0b001, 0b010, 0b011, 0b100, 0b101, 0b110, 0b111>
    {
      public:
        using Base = Multivector<T, 0b000, 0b001, 0b010, 0b011, 0b100, 0b101, 0b110, 0b111>;

        using Imaginary = Multivector<T, idx::ek, idx::i, idx::ej, idx::j, idx::ei, idx::k>;

        class Dual;

        class Primal : public Multivector<T, idx::scalar, idx::i, idx::j, idx::k>
        {
          public:
            using Imaginary = Multivector<T, idx::i, idx::j, idx::k>;

            Dual toDual() const
            {
                Dual dual;

                dual.template set<idx::e>(this->template get<idx::scalar>());
                dual.template set<idx::ei>(this->template get<idx::i>());
                dual.template set<idx::ej>(this->template get<idx::j>());
                dual.template set<idx::ek>(this->template get<idx::k>());

                return dual;
            }

            Imaginary getImaginary() const
            {
                Imaginary imaginary;

                imaginary.template set<idx::i>(this->template get<idx::i>());
                imaginary.template set<idx::j>(this->template get<idx::j>());
                imaginary.template set<idx::k>(this->template get<idx::k>());

                return imaginary;
            }
        };

        class Dual : public Multivector<T, idx::ek, idx::ej, idx::ei, idx::e>
        {
          public:
            using Imaginary = Multivector<T, idx::ek, idx::ej, idx::ei>;

            Primal toPrimal() const
            {
                Primal primal;

                primal.template set<idx::scalar>(this->template get<idx::e>());
                primal.template set<idx::i>(this->template get<idx::ei>());
                primal.template set<idx::j>(this->template get<idx::ej>());
                primal.template set<idx::k>(this->template get<idx::ek>());

                return primal;
            }

            Imaginary getImaginary() const
            {
                Imaginary imaginary;

                imaginary.template set<idx::ek>(this->template get<idx::ek>());
                imaginary.template set<idx::ej>(this->template get<idx::ej>());
                imaginary.template set<idx::ei>(this->template get<idx::ei>());

                return imaginary;
            }
        };

        using Base::Base;

        DualQuaternion();

        DualQuaternion(const Base &dq);

        DualQuaternion(Base &&dq);

        DualQuaternion(const Eigen::Quaternion<T> &quaternion);

        DualQuaternion(const Motor<T> &motor);

        DualQuaternion(const Point<T> &point);

        DualQuaternion(const Line<T> &plane);

        DualQuaternion(const Plane<T> &plane);

        virtual ~DualQuaternion();

        operator Motor<T>() const;

        using Base::vector;

        DualQuaternion conjugate() const;

        DualQuaternion sharpConjugate() const;

        DualQuaternion sharp() const;

        Primal getPrimal() const;

        Dual getDual() const;

        Imaginary getImaginary() const;

        const Base &mv() const
        {
            return *this;
        }

      protected:
      private:
    };

}  // namespace gafro::dqa

#include <gafro/algebra/dqa/DualQuaternion.hxx>