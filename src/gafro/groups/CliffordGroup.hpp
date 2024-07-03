#pragma once

#include <gafro/groups/LieGroup.hpp>

namespace gafro::groups
{

    class CliffordGroup : public LieGroup
    {
      public:
        CliffordGroup();

        virtual ~CliffordGroup();

      protected:
      private:
    };

}  // namespace gafro::groups