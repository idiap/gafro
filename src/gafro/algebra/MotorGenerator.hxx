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

#include <gafro/algebra/MotorGenerator.hpp>

namespace gafro
{

    template <class T>
    Motor<T>::Generator::Generator() : Base()
    {}

    template <class T>
    Motor<T>::Generator::Generator(const Base &other) : Base(other)
    {}

    template <class T>
    Motor<T>::Generator::Generator(const Parameters &parameters) : Base(parameters)
    {}

    template <class T>
    Motor<T>::Generator::Generator(const Eigen::Matrix<T, 3, 1> &p1, const Eigen::Matrix<T, 3, 1> &p2)
      : Motor<T>::Generator::Generator((Parameters() << p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]).finished())
    {}

    // template <class T>
    // template <class E>
    // Motor<T>::Generator::Generator(const Expression<E, Generator> &expression)
    // {
    //     Parameters parameters;

    //     parameters.coeffRef(0, 0) = expression.template get<blades::e23>();
    //     parameters.coeffRef(1, 0) = expression.template get<blades::e13>();
    //     parameters.coeffRef(2, 0) = expression.template get<blades::e12>();
    //     parameters.coeffRef(3, 0) = expression.template get<blades::e1i>();
    //     parameters.coeffRef(4, 0) = expression.template get<blades::e2i>();
    //     parameters.coeffRef(5, 0) = expression.template get<blades::e3i>();

    //     this->setParameters(std::move(parameters));
    // }

    template <class T>
    typename Rotor<T>::Generator Motor<T>::Generator::getRotorGenerator() const
    {
        return typename Rotor<T>::Generator({ this->template get<blades::e23>(),  //
                                              this->template get<blades::e13>(),  //
                                              this->template get<blades::e12>() });
    }

    template <class T>
    typename Translator<T>::Generator Motor<T>::Generator::getTranslatorGenerator() const
    {
        return typename Translator<T>::Generator({ this->template get<blades::e1i>(),  //
                                                   this->template get<blades::e2i>(),  //
                                                   this->template get<blades::e3i>() });
    }

}  // namespace gafro