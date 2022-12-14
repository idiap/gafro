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

#include <gafro/algebra/expressions/Cast.hpp>
#include <gafro/algebra/expressions/Dual.hpp>
#include <gafro/algebra/expressions/Inverse.hpp>
#include <gafro/algebra/expressions/Reverse.hpp>
//
#include <gafro/algebra/Blades.hpp>
//
#include <gafro/algebra/Multivector.hpp>

namespace gafro
{

    template <class T, int... index>
    Multivector<T, index...>::Multivector() : Multivector(Parameters::Zero())
    {}

    template <class T, int... index>
    Multivector<T, index...>::Multivector(const int &value) : Multivector(Parameters::Zero())
    {}

    template <class T, int... index>
    Multivector<T, index...>::Multivector(const Parameters &parameters) : parameters_(parameters)
    {}

    template <class T, int... index>
    Multivector<T, index...>::Multivector(const Multivector &other) : Multivector(other.parameters_)
    {}

    template <class T, int... index>
    Multivector<T, index...>::Multivector(Multivector &&other) : parameters_(std::forward<Parameters>(other.parameters_))
    {}

    template <class T, int... index>
    void Multivector<T, index...>::setParameters(Parameters &&parameters)
    {
        parameters_.noalias() = parameters;
    }

    template <class T, int... index>
    void Multivector<T, index...>::setParameters(const Parameters &parameters)
    {
        parameters_.noalias() = parameters;
    }

    template <class T, int... index>
    typename Multivector<T, index...>::Parameters &Multivector<T, index...>::vector()
    {
        return parameters_;
    }

    template <class T, int... index>
    const typename Multivector<T, index...>::Parameters &Multivector<T, index...>::vector() const
    {
        return parameters_;
    }

    template <class T, int... index>
    constexpr const Bitset<index...> &Multivector<T, index...>::bits()
    {
        return bits_;
    }

    template <class T, int... index>
    constexpr const std::array<int, Multivector<T, index...>::size> &Multivector<T, index...>::blades()
    {
        return blades_;
    }

    template <class T, int... index>
    constexpr bool Multivector<T, index...>::has(const int &blade)
    {
        return bits_.test(blade);
    }

    template <class T, int... index>
    Reverse<Multivector<T, index...>> Multivector<T, index...>::reverse() const
    {
        return Reverse<Multivector>(*static_cast<const Multivector *>(this));
    }

    template <class T, int... index>
    Inverse<Multivector<T, index...>> Multivector<T, index...>::inverse() const
    {
        return Inverse<Multivector>(*static_cast<const Multivector *>(this));
    }

    template <class T, int... index>
    Dual<Multivector<T, index...>> Multivector<T, index...>::dual() const
    {
        return Dual<Multivector>(*static_cast<const Multivector *>(this));
    }

    template <class T, int... index>
    T Multivector<T, index...>::norm() const
    {
        return sqrt(abs(squaredNorm()));
    }

    template <class T, int... index>
    T Multivector<T, index...>::squaredNorm() const
    {
        return ((*this) * this->reverse()).template get<blades::scalar>();
    }

    template <class T, int... index>
    T Multivector<T, index...>::signedNorm() const
    {
        T squared_norm = squaredNorm();

        return squared_norm > 0 ? sqrt(squared_norm) : -sqrt(abs(squared_norm));
    }

    template <class T, int... index>
    void Multivector<T, index...>::normalize()
    {
        T value = norm();

        if (value > 1e-10)
        {
            *this = (*this) * Scalar<T>(1.0 / value);
        }
    }

    // template <class T, int... index>
    // const T &Multivector<T, index...>::operator[](int blade) const
    // {
    //     const static T zero = 0.0;

    //     if (has(blade))
    //     {
    //         return parameters_.coeff(map(blade), 0);
    //     }
    //     else
    //     {
    //         return zero;
    //     }
    // }

    template <class T, int... index>
    template <std::size_t... i>
    constexpr auto Multivector<T, index...>::getBlades(std::index_sequence<i...>)
    {
        return std::tuple{ Multivector<T, blades()[i]>()... };
    }

    template <class T, int... index>
    constexpr auto Multivector<T, index...>::split()
    {
        return getBlades(std::make_index_sequence<size>{});
    }

    template <class T, int... index>
    Multivector<T, index...> Multivector<T, index...>::Random()
    {
        return Multivector(Parameters::Random());
    }

    template <class T, int... index>
    template <class Other>
    Other Multivector<T, index...>::cast() const
    {
        return Cast<Multivector<T, index...>, Other>(*this);
    }

    template <class T, int... index>
    std::ostream &operator<<(std::ostream &ostream, const Multivector<T, index...> &mv)
    {
        static const std::array<std::string, 32> BladeNames = { "",     "e1",   "e2",    "e3",    "ei",    "e0",    "e23",   "e13",
                                                                "e12",  "e1i",  "e2i",   "e3i",   "e01",   "e02",   "e03",   "e0i",
                                                                "e123", "e12i", "e13i",  "e23i",  "e012",  "e013",  "e023",  "e01i",
                                                                "e02i", "e03i", "e123i", "e0123", "e012i", "e023i", "e013i", "e0123i" };

        if (sizeof...(index) == 0)
        {
            ostream << 0;
        }

        bool first = true;

        for (unsigned int k = 0; k < mv.vector().rows(); ++k)
        {
            if (std::abs(mv.vector().coeff(k, 0)) < 1e-10)
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

            if (Multivector<T, index...>::blades()[k] > 0)
            {
                ostream << "*" << BladeNames[Multivector<T, index...>::blades()[k]];
            }
        }

        if (first)
        {
            ostream << 0;
        }

        return ostream;
    }

}  // namespace gafro