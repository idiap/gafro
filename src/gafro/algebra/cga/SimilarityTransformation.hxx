// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/SimilarityTransformation.hpp>

namespace gafro
{

    template <typename T>
    SimilarityTransformation<T>::SimilarityTransformation() : Base(Scalar<T>::One() + Base::Zero())
    {}

    template <typename T>
    SimilarityTransformation<T>::SimilarityTransformation(const Generator &generator) : Base(SimilarityTransformation::exp(generator))  //
    {}

    template <typename T>
    SimilarityTransformation<T>::~SimilarityTransformation() = default;

    template <typename T>
    typename SimilarityTransformation<T>::CanonicalDecomposition SimilarityTransformation<T>::getCanonicalDecomposition() const
    {
        return CanonicalDecomposition(*this);
    }

    template <typename T>
    typename SimilarityTransformation<T>::Generator SimilarityTransformation<T>::log() const
    {
        if (this->norm() < 1e-5)
        {
            return Generator::Zero();
        }

        CanonicalDecomposition decomposition = this->getCanonicalDecomposition();

        return decomposition.getDilator().logarithm() +  //
               decomposition.getTranslator().log() +     //
               decomposition.getRotor().log();
    }

    template <typename T>
    SimilarityTransformation<T>::CanonicalDecomposition::CanonicalDecomposition(const SimilarityTransformation &transformation)
    {
        Ei<T> ei(TypeTraits<T>::Value(1.0));

        dilator_ = transformation.apply(ei).template get<blades::ei>();
        Motor<T> motor = transformation * dilator_.reverse();
        rotor_ = motor.getRotor();
        translator_ = motor.getTranslator();
    }

    template <typename T>
    const Dilator<T> &SimilarityTransformation<T>::CanonicalDecomposition::getDilator() const
    {
        return dilator_;
    }

    template <typename T>
    const Rotor<T> &SimilarityTransformation<T>::CanonicalDecomposition::getRotor() const
    {
        return rotor_;
    }

    template <typename T>
    const Translator<T> &SimilarityTransformation<T>::CanonicalDecomposition::getTranslator() const
    {
        return translator_;
    }

    template <typename T>
    SimilarityTransformation<T> SimilarityTransformation<T>::exp(const typename Generator::Parameters &generator)
    {
        return SimilarityTransformation<T>::exp(Generator(generator));
    }

    template <typename T>
    SimilarityTransformation<T> SimilarityTransformation<T>::exp(const Generator &generator)
    {
        return Translator<T>::exp(generator.template extract<blades::e1i>() + generator.template extract<blades::e2i>() +
                                  generator.template extract<blades::e3i>()) *
               Rotor<T>::exp(generator.template extract<blades::e12>() + generator.template extract<blades::e13>() +
                             generator.template extract<blades::e23>()) *
               Dilator<T>::exp(generator.template extract<blades::e0i>());
    }

}  // namespace gafro