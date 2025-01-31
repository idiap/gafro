#pragma once

#include <gafro/algebra/Algebra.hpp>

namespace gafro
{

    template <int p, int q, int r>
    struct Metric
    {
        constexpr static int dim = p + q + r;

        template <int i, int j>
        constexpr static double get()
        {
            if constexpr ((i == j) && (i >= 0))
            {
                if constexpr (i < p)
                {
                    return 1.0;
                }

                if constexpr ((i >= p) && (i < p + q))
                {
                    return -1.0;
                }
            }

            return 0.0;
        }

        using Algebra = gafro::Algebra<Metric>;

      private:
        template <int... j, int... k>
        constexpr static void fillTensor(Eigen::Matrix<double, dim, dim> &tensor, std::index_sequence<j...>, std::index_sequence<k...>)
        {
            (tensor.coeffRef(j, k), ...) = (get<j, k>(), ...);
        }

      public:
        constexpr static Eigen::Matrix<double, dim, dim> getTensor()
        {
            Eigen::Matrix<double, dim, dim> tensor;

            fillTensor(tensor, std::make_index_sequence<dim>(), std::make_index_sequence<dim>());

            return tensor;
        }
    };

}  // namespace gafro