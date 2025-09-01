#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
//
#include <gafro/gafro.hpp>

namespace gafro
{
    template <>
    struct TypeTraits<autodiff::dual>
    {
        static autodiff::dual Zero()
        {
            return 0.0;
        }

        static autodiff::dual One()
        {
            return 1.0;
        }

        static autodiff::dual Value(const double &value)
        {
            return value;
        }

        static autodiff::dual copy(const autodiff::dual &value)
        {
            return value;
        }

        static bool greater(const autodiff::dual &v1, const autodiff::dual &v2)
        {
            return v1 > v2;
        }

        static bool greaterEqual(const autodiff::dual &v1, const autodiff::dual &v2)
        {
            return v1 >= v2;
        }
    };

    template <>
    struct TypeTraits<autodiff::dual2nd>
    {
        static autodiff::dual2nd Zero()
        {
            return 0.0;
        }

        static autodiff::dual2nd One()
        {
            return 1.0;
        }

        static autodiff::dual2nd Value(const double &value)
        {
            return value;
        }

        static autodiff::dual2nd copy(const autodiff::dual2nd &value)
        {
            return value;
        }

        static bool greater(const autodiff::dual2nd &v1, const autodiff::dual2nd &v2)
        {
            return v1 > v2;
        }

        static bool greaterEqual(const autodiff::dual2nd &v1, const autodiff::dual2nd &v2)
        {
            return v1 >= v2;
        }
    };

    namespace autodiff
    {
        using Point = gafro::Point<::autodiff::dual>;
        using Line = gafro::Line<::autodiff::dual>;
        using Circle = gafro::Circle<::autodiff::dual>;
        using Plane = gafro::Plane<::autodiff::dual>;
        using Sphere = gafro::Sphere<::autodiff::dual>;
        using Rotor = gafro::Rotor<::autodiff::dual>;
        using Translator = gafro::Translator<::autodiff::dual>;
        using Motor = gafro::Motor<::autodiff::dual>;
    }  // namespace autodiff

}  // namespace gafro