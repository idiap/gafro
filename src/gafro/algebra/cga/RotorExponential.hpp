// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/UnaryExpression.hpp>
#include <gafro/algebra/cga/Rotor.hpp>

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