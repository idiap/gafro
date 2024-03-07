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

namespace gafro
{

    template <class T>
    struct TypeTraits
    {
        static T Zero();

        static T One();

        static T Value(const double &value);

        static T copy(const T &value);

        static bool greater(const T &v1, const T &v2);
    };

    template <>
    struct TypeTraits<double>
    {
        static double Zero()
        {
            return 0.0;
        }

        static double One()
        {
            return 1.0;
        }

        static double Value(const double &value)
        {
            return value;
        }

        static double copy(const double &value)
        {
            return value;
        }

        static bool greater(const double &v1, const double &v2)
        {
            return v1 > v2;
        }

        static bool greaterEqual(const double &v1, const double &v2)
        {
            return v1 >= v2;
        }
    };

}  // namespace gafro