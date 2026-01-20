#pragma once

#include <Eigen/Core>
//
#include <gafro/groups/LieGroup.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r>
    class MatrixLieGroup : public LieGroup<T, p, q, r>
    {
      public:
        using LieGroup<T, p, q, r>::dimension;
        using Element = Eigen::Matrix<T, dimension, dimension>;

      public:
        MatrixLieGroup();

        virtual ~MatrixLieGroup();

      protected:
      private:
    };

}  // namespace gafro::groups

#include <gafro/groups/MatrixLieGroup.hxx>