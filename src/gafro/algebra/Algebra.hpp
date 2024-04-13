#pragma once

#include <cmath>
#include <gafro/algebra/util/Bitset.hpp>

namespace gafro
{

    namespace math
    {
        template <int k>
        constexpr int pow2()
        {
            return 2 * pow2<k - 1>();
        }

        template <>
        constexpr int pow2<0>()
        {
            return 1;
        }

        template <int k>
        constexpr int pown()
        {
            return -1 * pown<k - 1>();
        }

        template <>
        constexpr int pown<0>()
        {
            return 1;
        }

        template <int k>
        constexpr int positive()
        {
            if constexpr (k < 0)
            {
                return 0;
            }

            return k;
        }

    }  // namespace math

    template <class M1, class M2, template <class V> class Operation>
    class Sum;

    namespace detail
    {
        template <class T>
        class AdditionOperator;
    }

    template <class M>
    class Algebra
    {
      public:
        using Metric = M;

        template <class T, int... index>
        class Multivector;

        constexpr static int dim = math::pow2<M::dim>();

        class BladeBitmap
        {
          public:
            template <int i>
            constexpr static int getGrade()
            {
                if constexpr (i != 0)
                {
                    return 1 + getGrade<(i & (i - 1))>();
                }

                return 0;
            }

            template <int b1, int b2>
            constexpr static int getGrade()
            {
                constexpr int g1 = getGrade<b1>();
                constexpr int g2 = getGrade<b2>();

                if constexpr (g1 - g2 > 0)
                {
                    return g1 - g2;
                }
                else
                {
                    return g2 - g1;
                }
            }

            template <int b1, int b2>
            constexpr static int getLeftShifts()
            {
                if constexpr (b1 != 0)
                {
                    if constexpr ((b1 & b2) > 0)
                    {
                        return 1 + getLeftShifts<(b1 >> 1), b2>();
                    }
                    else
                    {
                        return getLeftShifts<(b1 >> 1), b2>();
                    }
                }

                return -1;  // don't count first call, as it doesn't shift a bit
            }

            template <int b1, int b2>
            constexpr static int getRightShifts()
            {
                if constexpr (b1 <= dim)
                {
                    if constexpr ((b1 & b2) > 0)
                    {
                        return 1 + getRightShifts<(b1 << 1), b2>();
                    }
                    else
                    {
                        return getRightShifts<(b1 << 1), b2>();
                    }
                }

                return -1;  // don't count first call, as it doesn't shift a bit
            }

            /**
             * @brief count the number of basis vector swaps required to get 'b1' and 'b2' into canonical order
             * @details [long description]
             *
             * @tparam b1 [description]
             * @tparam b2 [description]
             * @tparam i [description]
             * @return [description]
             */
            template <int b1, int b2, int i>
            constexpr static int countSwaps()
            {
                if constexpr (b1 >> i != 0)
                {
                    return getGrade<(b1 >> i) & b2>() + countSwaps<b1, b2, i + 1>();
                }

                return 0.0;
            }

            template <int b1, int b2>
            constexpr static double reorderSign()
            {
                constexpr int sum = countSwaps<b1, b2, 1>();

                // even number of swaps -> return 1
                // odd number of swaps -> return -1
                return ((sum & 1) == 0) ? 1.0 : -1.0;
            }
        };

        template <class T, int i1, int i2>
        class GeometricProduct;

        template <class T, int i1, int i2>
        class OuterProduct;

        template <class T, int i1, int i2>
        class InnerProduct;

        template <class T>
        using Pseudoscalar = Multivector<T, dim - 1>;
    };

}  // namespace gafro

#include <gafro/algebra/expressions/GeometricProductCayleyTable.hpp>
#include <gafro/algebra/expressions/InnerProductCayleyTable.hpp>
#include <gafro/algebra/expressions/OuterProductCayleyTable.hpp>