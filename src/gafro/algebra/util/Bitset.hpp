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

#include <array>
#include <bitset>

namespace gafro
{
    template <int dim, int... i>
    struct Bitset;

    template <int dim, int i0, int... i>
    struct Bitset<dim, i0, i...>
    {
        constexpr static int size()
        {
            int s = 0;

            for (unsigned k = 0; k < dim; ++k)
            {
                if (bits[k])
                {
                    s += 1;
                }
            }

            return s;
        }

        constexpr static bool test(int k)
        {
            return bits[k];
        }

        constexpr static std::array<bool, dim> bits = [] {
            std::array<bool, dim> bits;

            for (unsigned k = 0; k < dim; ++k)
            {
                bits[k] = Bitset<dim, i0>().bits[k] || Bitset<dim, i...>().bits[k];
            }

            return bits;
        }();

        constexpr static std::array<int, size()> blades()
        {
            std::array<int, size()> array;

            int j = 0;

            for (unsigned k = 0; k < dim; ++k)
            {
                if (test(k))
                {
                    array[j++] = k;
                }
            }

            return array;
        }
    };

    template <int dim, int i>
    struct Bitset<dim, i>
    {
        constexpr static int size()
        {
            return 1;
        }

        constexpr static bool test(int k)
        {
            return (i == k);
        }

        constexpr static std::array<bool, dim> bits = [] {
            std::array<bool, dim> bits;
            for (unsigned k = 0; k < dim; ++k)
            {
                bits[k] = false;
            }
            bits[i] = true;
            return bits;
        }();

        constexpr static std::array<int, 1> blades()
        {
            std::array<int, 1> array;

            array[0] = i;

            return array;
        }
    };

    template <int dim>
    struct Bitset<dim>
    {
        constexpr static int size()
        {
            return 0;
        }

        constexpr static bool test(int k)
        {
            return false;
        }

        constexpr static std::array<bool, dim> bits = [] {
            std::array<bool, dim> bits;
            for (unsigned k = 0; k < dim; ++k)
            {
                bits[k] = false;
            }
            return bits;
        }();

        constexpr static std::array<int, 0> blades()
        {
            std::array<int, 0> array{};

            return array;
        }
    };

    template <int dim, int... i>
    std::ostream &operator<<(std::ostream &ostream, const Bitset<dim, i...> &bitset)
    {
        for (unsigned k = 0; k < dim; ++k)
        {
            ostream << bitset.bits[k];
        }

        return ostream;
    }

    template <int dim, int... i, int... j>
    constexpr auto operator|(const Bitset<dim, i...> &b1, const Bitset<dim, j...> &b2)
    {
        return Bitset<dim, i..., j...>();
    }

}  // namespace gafro