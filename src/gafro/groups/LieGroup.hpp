#pragma once

#include <gafro/groups/Group.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r>
    class LieGroup : public Group<T, p, q, r>
    {
      public:
        using Group<T, p, q, r>::dimension;

      public:
        LieGroup() = default;

        virtual ~LieGroup() = default;

      protected:
      private:
    };

}  // namespace gafro::groups