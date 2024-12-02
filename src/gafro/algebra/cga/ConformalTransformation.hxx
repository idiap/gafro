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

#include <gafro/algebra/cga/ConformalTransformation.hpp>
#include <gafro/algebra/cga/Dilator.hxx>
#include <gafro/algebra/cga/Rotor.hxx>
#include <gafro/algebra/cga/Translator.hxx>
#include <gafro/algebra/cga/Transversion.hpp>

namespace gafro
{

    template <typename T>
    ConformalTransformation<T>::ConformalTransformation() : Base(Scalar<T>::One() + Base::Zero())
    {}

    template <typename T>
    ConformalTransformation<T>::ConformalTransformation(const Generator &generator)
      : Base(Scalar<T>(TypeTraits<T>::One()) + Scalar<T>(TypeTraits<T>::Value(-0.5)) * generator)  //
    {}

    template <typename T>
    ConformalTransformation<T>::~ConformalTransformation() = default;

    template <typename T>
    typename ConformalTransformation<T>::Generator ConformalTransformation<T>::log() const
    {
        if (this->norm() < 1e-5)
        {
            return Generator::Zero();
        }

        Dilator<T> dilator = this->apply(Ei<T>(1.0)).template get<blades::ei>();
        auto str = (dilator.reverse() * (*this)).evaluate();
        Rotor<T> rotor = (Ei<T>(1.0) | (E0<T>(1.0) ^ (E0<T>(1.0) | (str * Ei<T>(1.0)))));
        auto st = (str * rotor.reverse()).evaluate();
        Transversion<T> transversion = (st * Ei<T>(1.0)) | E0<T>(-1.0);
        Translator<T> translator = transversion.reverse() * st;

        return dilator.log() + transversion.log() + translator.log() + rotor.log();
    }

    template <typename T>
    ConformalTransformation<T> ConformalTransformation<T>::exp(const Generator::Parameters &generator)
    {
        return ConformalTransformation<T>::exp(Generator(generator));
    }

    template <typename T>
    ConformalTransformation<T> ConformalTransformation<T>::exp(const Generator &generator)
    {
        return Dilator<T>::exp(generator.template extract<blades::e0i>()) *
               Transversion<T>::exp(generator.template extract<blades::e01>() + generator.template extract<blades::e02>() +
                                    generator.template extract<blades::e03>()) *
               Translator<T>::exp(generator.template extract<blades::e1i>() + generator.template extract<blades::e2i>() +
                                  generator.template extract<blades::e3i>()) *
               Rotor<T>::exp(generator.template extract<blades::e12>() + generator.template extract<blades::e13>() +
                             generator.template extract<blades::e23>());
    }

}  // namespace gafro