#pragma once

#include <gafro/algebra/Metric.hpp>
#include <gafro/groups/LieGroup.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r>
    class CliffordGroup : public LieGroup<T, p, q, r>
    {
      public:
        constexpr static int dimension = Group<T, p, q, r>::dimension;
        using Algebra = Metric<p, q, r>::Algebra;
        using Vector = typename Algebra::template Grade<T, 1>;

      public:
        CliffordGroup() = default;

        virtual ~CliffordGroup() = default;

      protected:
      private:
    };

}  // namespace gafro::groups