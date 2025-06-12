// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

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

        static constexpr bool is_scalar_type = false;
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

        static constexpr bool is_scalar_type = true;
    };

}  // namespace gafro