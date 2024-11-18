#pragma once

#include <gafro/groups/CoveringGroup.hpp>
#include <gafro/groups/PinGroup.hpp>
#include <gafro/groups/Subgroup.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r>
    class SpinGroup : public Subgroup<T, p, q, r, PinGroup<T, p, q, r>>
    {
      public:
        constexpr static int dimension = Group<T, p, q, r>::dimension;
        using typename PinGroup<T, p, q, r>::Algebra;
        using Vector = typename PinGroup<T, p, q, r>::Vector;

      public:
        SpinGroup() = default;

        virtual ~SpinGroup() = default;

        template <int k>
            requires(k % 2 == 0)
        using Element = typename PinGroup<T, p, q, r>::template Element<k>;

      protected:
      private:
    };

}  // namespace gafro::groups