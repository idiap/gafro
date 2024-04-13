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

#include <gafro/algebra/expressions/Assign.hpp>
#include <gafro/algebra/expressions/Cast.hpp>
#include <gafro/algebra/expressions/CommutatorProduct.hpp>
#include <gafro/algebra/expressions/Dual.hpp>
#include <gafro/algebra/expressions/Inverse.hpp>
#include <gafro/algebra/expressions/Reverse.hpp>
//
#include <gafro/algebra/Blades.hpp>
//
#include <gafro/algebra/Multivector.hpp>

namespace gafro
{

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector() : Multivector(Parameters::Constant(TypeTraits<T>::Zero()))
    {}

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector(const int &value) : Multivector(Parameters::Ones() * value)
    {}

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector(const Parameters &parameters) : parameters_(parameters)
    {}

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector(Parameters &&parameters) : parameters_(std::move(parameters))
    {}

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector(const Multivector &other) : parameters_(other.parameters_)
    {}

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector(Multivector &&other) : parameters_(std::move(other.parameters_))
    {}

    template <class M>
    template <class T, int... index>
    template <class Derived>
    Algebra<M>::Multivector<T, index...>::Multivector(const Expression<Derived, Multivector> &expression)
    {
        *this = expression;
    }

    template <class M>
    template <class T, int... index>
    template <class Derived, class Other>
    Algebra<M>::Multivector<T, index...>::Multivector(const Expression<Derived, Other> &expression)
    {
        *this = expression;
    }

    template <class M>
    template <class T, int... index>
    template <class S>
    Algebra<M>::Multivector<T, index...>::Multivector(const typename Algebra<M>::template Multivector<S, index...> &other)
      : parameters_(other.vector().unaryExpr([](const S &v) { return TypeTraits<T>::Value(v); }))
    {}

    // OPERATORS

    template <class M>
    template <class T, int... index>
    void Algebra<M>::Multivector<T, index...>::setParameters(Parameters &&parameters)
    {
        parameters_.noalias() = std::move(parameters);
    }

    template <class M>
    template <class T, int... index>
    void Algebra<M>::Multivector<T, index...>::setParameters(const Parameters &parameters)
    {
        parameters_.noalias() = parameters;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...>::Parameters &Algebra<M>::Multivector<T, index...>::vector()
    {
        return parameters_;
    }

    template <class M>
    template <class T, int... index>
    const typename Algebra<M>::template Multivector<T, index...>::Parameters &Algebra<M>::Multivector<T, index...>::vector() const
    {
        return parameters_;
    }

    //

    template <class M>
    template <class T, int... index>
    Reverse<typename Algebra<M>::template Multivector<T, index...>> Algebra<M>::Multivector<T, index...>::reverse() const
    {
        return Reverse<typename Algebra<M>::template Multivector<T, index...>>(*this);
    }

    template <class M>
    template <class T, int... index>
    Inverse<typename Algebra<M>::template Multivector<T, index...>> Algebra<M>::Multivector<T, index...>::inverse() const
    {
        return Inverse<typename Algebra<M>::template Multivector<T, index...>>(*this);
    }

    template <class M>
    template <class T, int... index>
    Dual<typename Algebra<M>::template Multivector<T, index...>> Algebra<M>::Multivector<T, index...>::dual() const
    {
        return Dual<typename Algebra<M>::template Multivector<T, index...>>(*this);
    }

    // OPERATORS

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(const Parameters &parameters)
    {
        setParameters(parameters);

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(Parameters &&parameters)
    {
        setParameters(std::move(parameters));

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(const Multivector &other)
    {
        setParameters(other.parameters_);

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(Multivector &&other)
    {
        setParameters(std::move(other.parameters_));

        return *this;
    }

    template <class M>
    template <class T, int... index>
    template <class Derived>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(
      const Expression<Derived, Multivector> &expression)
    {
        Assign<Derived, Multivector, T, M, index...>::run(*this, expression);

        return *this;
    }

    template <class M>
    template <class T, int... index>
    template <class Derived, class Other>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(
      const Expression<Derived, Other> &expression)
    {
        Assign<Derived, Other, T, M, index...>::run(*this, expression);

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator*=(const T &scalar)
    {
        parameters_ *= scalar;

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator/=(const T &scalar)
    {
        parameters_ /= scalar;

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator+=(const Multivector &other)
    {
        parameters_ += other.parameters_;

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::operator-() const
    {
        return Multivector(-this->vector());
    }

    //

    template <class M>
    template <class T, int... index>
    constexpr const Bitset<Algebra<M>::dim, index...> &Algebra<M>::Multivector<T, index...>::bits()
    {
        return bits_;
    }

    template <class M>
    template <class T, int... index>
    constexpr const std::array<int, Algebra<M>::template Multivector<T, index...>::size> &Algebra<M>::Multivector<T, index...>::blades()
    {
        return blades_;
    }

    template <class M>
    template <class T, int... index>
    constexpr bool Algebra<M>::Multivector<T, index...>::has(const int &blade)
    {
        return bits_.test(blade);
    }

    //

    template <class M>
    template <class T, int... index>
    template <class M2>
    CommutatorProduct<typename Algebra<M>::template Multivector<T, index...>, M2> Algebra<M>::Multivector<T, index...>::commutatorProduct(
      const M2 &multivector) const
    {
        return CommutatorProduct<typename Algebra<M>::template Multivector<T, index...>, M2>(*this, multivector);
    }

    //

    template <class M>
    template <class T, int... index>
    T Algebra<M>::Multivector<T, index...>::norm() const
    {
        return sqrt(abs(squaredNorm()));
    }

    template <class M>
    template <class T, int... index>
    T Algebra<M>::Multivector<T, index...>::squaredNorm() const
    {
        return ((*this) * this->reverse()).template get<blades::scalar>();
    }

    template <class M>
    template <class T, int... index>
    T Algebra<M>::Multivector<T, index...>::signedNorm() const
    {
        T squared_norm = squaredNorm();

        return squared_norm > 0 ? sqrt(squared_norm) : -sqrt(abs(squared_norm));
    }

    template <class M>
    template <class T, int... index>
    void Algebra<M>::Multivector<T, index...>::normalize()
    {
        T value = norm();

        // if (value > 1e-10)
        // {
        *this = (*this) * Scalar<T>(1.0 / value);
        // }
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::normalized() const
    {
        Multivector other = *this;

        other.normalize();

        return other;
    }

    //

    template <class M>
    template <class T, int... index>
    template <std::size_t... i>
    constexpr auto Algebra<M>::Multivector<T, index...>::getBlades(std::index_sequence<i...>, const Multivector &multivector)
    {
        return std::tuple{ Blade<T, blades()[i]>(multivector.get<blades()[i]>())... };
    }

    template <class M>
    template <class T, int... index>
    auto Algebra<M>::Multivector<T, index...>::getBlades() const
    {
        return getBlades(std::make_index_sequence<size>{}, *this);
    }

    //

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::Random()
    {
        return Multivector(Parameters::Random());
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::Zero()
    {
        return Multivector(Parameters::Constant(TypeTraits<T>::Zero()));
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::One()
    {
        return Multivector(Parameters::Ones());
    }

    template <class M>
    template <class T, int... index>
    template <class Other>
    Other Algebra<M>::Multivector<T, index...>::cast() const
    {
        return Cast<typename Algebra<M>::template Multivector<T, index...>, Other>(*this);
    }

    //

    template <int... index>
    std::ostream &operator<<(std::ostream &ostream, const ConformalGeometricAlgebra::Multivector<double, index...> &mv)
    {
        static const std::array<std::string, 32> BladeNames = { "",    "e0",   "e1",   "e01",   "e2",   "e02",   "e12",   "e012",
                                                                "e3",  "e03",  "e13",  "e013",  "e23",  "e023",  "e123",  "e0123",
                                                                "ei",  "e0i",  "e1i",  "e01i",  "e2i",  "e02i",  "e12i",  "e012i",
                                                                "e3i", "e03i", "e13i", "e013i", "e23i", "e023i", "e123i", "e0123i" };

        if (sizeof...(index) == 0)
        {
            ostream << 0;

            return ostream;
        }

        bool first = true;

        for (unsigned int k = 0; k < mv.vector().rows(); ++k)
        {
            if (abs(mv.vector().coeff(k, 0)) < 1e-10)
            {
                continue;
            }

            if (!first)
            {
                ostream << (mv.vector().coeff(k, 0) >= 0 ? " + " : " - ");

                ostream << abs(mv.vector().coeff(k, 0));
            }
            else
            {
                ostream << mv.vector().coeff(k, 0);

                first = false;
            }

            if (ConformalGeometricAlgebra::Multivector<double, index...>::blades()[k] > 0)
            {
                ostream << "*" << BladeNames[ConformalGeometricAlgebra::Multivector<double, index...>::blades()[k]];
            }
        }

        if (first)
        {
            ostream << 0;
        }

        return ostream;
    }

}  // namespace gafro