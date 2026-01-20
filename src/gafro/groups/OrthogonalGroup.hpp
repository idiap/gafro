#pragma once

#include <gafro/groups/MatrixLieGroup.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r>
    class OrthogonalGroup : public MatrixLieGroup<T, p, q, r>
    {
      public:
        OrthogonalGroup() = default;

        virtual ~OrthogonalGroup() = default;

      protected:
      private:
    };

}  // namespace gafro::groups