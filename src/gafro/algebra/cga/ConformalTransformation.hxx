// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/ConformalTransformation.hpp>

namespace gafro
{

    template <typename T>
    ConformalTransformation<T>::ConformalTransformation() : Base(Scalar<T>::One() + Base::Zero())
    {}

    template <typename T>
    ConformalTransformation<T>::ConformalTransformation(const Generator &generator) : Base(ConformalTransformation::exp(generator))
    {}

    template <typename T>
    ConformalTransformation<T>::~ConformalTransformation() = default;

    template <typename T>
    typename ConformalTransformation<T>::CanonicalDecomposition ConformalTransformation<T>::getCanonicalDecomposition() const
    {
        return CanonicalDecomposition(*this);
    }

    template <typename T>
    typename ConformalTransformation<T>::Generator ConformalTransformation<T>::log() const
    {
        if (this->norm() < 1e-5)
        {
            return Generator::Zero();
        }

        CanonicalDecomposition decomposition = this->getCanonicalDecomposition();

        return decomposition.getDilator().logarithm() +  //
               decomposition.getTransversion().log() +   //
               decomposition.getTranslator().log() +     //
               decomposition.getRotor().log();
    }

    template <typename T>
    ConformalTransformation<T>::CanonicalDecomposition::CanonicalDecomposition(const ConformalTransformation &transformation)
    {
        Ei<T> ei(TypeTraits<T>::Value(1.0));
        E0<T> e0(TypeTraits<T>::Value(1.0));

        dilator_ = transformation.apply(ei).template get<blades::ei>();
        auto str = (dilator_.reverse() * transformation).evaluate();
        rotor_ = (ei | (e0 ^ (e0 | (str * ei))));
        auto st = (str * rotor_.reverse()).evaluate();
        transversion_ = (st * ei) | -e0;
        translator_ = transversion_.reverse() * st;
    }

    template <typename T>
    const Dilator<T> &ConformalTransformation<T>::CanonicalDecomposition::getDilator() const
    {
        return dilator_;
    }

    template <typename T>
    const Rotor<T> &ConformalTransformation<T>::CanonicalDecomposition::getRotor() const
    {
        return rotor_;
    }

    template <typename T>
    const Transversion<T> &ConformalTransformation<T>::CanonicalDecomposition::getTransversion() const
    {
        return transversion_;
    }

    template <typename T>
    const Translator<T> &ConformalTransformation<T>::CanonicalDecomposition::getTranslator() const
    {
        return translator_;
    }

    template <typename T>
    ConformalTransformation<T> ConformalTransformation<T>::exp(const typename Generator::Parameters &generator)
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