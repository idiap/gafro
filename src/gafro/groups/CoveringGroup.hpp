#pragma once

#include <gafro/groups/Group.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r, DerivedGroup<T, p, q, r> Cover, DerivedGroup<T, p, q, r> Base>
    class CoveringGroup
    {
      public:
        CoveringGroup();

        virtual ~CoveringGroup();

      protected:
      private:
    };

}  // namespace gafro::groups