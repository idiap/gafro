#pragma once

#include <gafro/groups/Group.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r, DerivedGroup<T, p, q, r> Base>
    class CoveringGroup
    {
      public:
        CoveringGroup() = default;

        virtual ~CoveringGroup() = default;

      protected:
      private:
    };

}  // namespace gafro::groups