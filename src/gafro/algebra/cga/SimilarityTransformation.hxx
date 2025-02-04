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