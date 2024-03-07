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

#include <array>
#include <bitset>

namespace gafro
{
    template <int... i>
    struct Bitset;

    template <int i0, int... i>
    struct Bitset<i0, i...>
    {
        constexpr static int size()
        {
            int s = 0;

            for (unsigned k = 0; k < 32; ++k)
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

        constexpr static std::array<bool, 32> bits = [] {
            std::array<bool, 32> bits;

            for (unsigned k = 0; k < 32; ++k)
            {
                bits[k] = Bitset<i0>().bits[k] || Bitset<i...>().bits[k];
            }

            return bits;
        }();

        constexpr static std::array<int, size()> blades()
        {
            std::array<int, size()> array;

            int j = 0;

            for (unsigned k = 0; k < 32; ++k)
            {
                if (test(k))
                {
                    array[j++] = k;
                }
            }

            return array;
        }
    };

    template <int i>
    struct Bitset<i>
    {
        constexpr int size() const
        {
            return 1;
        }

        constexpr bool test(int k) const
        {
            return (i == k);
        }

        constexpr static std::array<bool, 32> bits = [] {
            std::array<bool, 32> bits;
            for (unsigned k = 0; k < 32; ++k)
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

    template <>
    struct Bitset<>
    {
        constexpr int size() const
        {
            return 0;
        }

        constexpr bool test(int k) const
        {
            return false;
        }

        constexpr static std::array<bool, 32> bits = [] {
            std::array<bool, 32> bits;
            for (unsigned k = 0; k < 32; ++k)
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

    template <int... i>
    std::ostream &operator<<(std::ostream &ostream, const Bitset<i...> &bitset)
    {
        for (unsigned k = 0; k < 32; ++k)
        {
            ostream << bitset.bits[k];
        }

        return ostream;
    }

    template <int... i, int... j>
    constexpr auto operator|(const Bitset<i...> &b1, const Bitset<j...> &b2)
    {
        return Bitset<i..., j...>();
    }

}  // namespace gafro