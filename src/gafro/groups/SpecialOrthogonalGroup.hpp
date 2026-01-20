#pragma once

#include <gafro/groups/OrthogonalGroup.hpp>
#include <gafro/groups/Subgroup.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r>
    class SpecialOrthogonalGroup : public Subgroup<T, p, q, r, OrthogonalGroup<T, p, q, r>>  //
    {
      public:
        SpecialOrthogonalGroup() = default;

        virtual ~SpecialOrthogonalGroup() = default;

      protected:
      private:
    };

}  // namespace gafro::groups