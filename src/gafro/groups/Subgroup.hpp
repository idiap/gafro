#pragma once

#include <gafro/groups/Group.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r, DerivedGroup<T, p, q, r> Supergroup>
    class Subgroup : public Supergroup
    {
      public:
        Subgroup();

        virtual ~Subgroup();

      protected:
      private:
    };

}  // namespace gafro::groups