#pragma once

#include <gafro/groups/CliffordGroup.hpp>
#include <gafro/groups/CoveringGroup.hpp>
#include <gafro/groups/OrthogonalGroup.hpp>
#include <gafro/groups/Subgroup.hpp>

namespace gafro::groups
{

    namespace pin_detail
    {
        template <class T, int p, int q, int r>
        using Vector = typename CliffordGroup<T, p, q, r>::Vector;

        template <class T, int p, int q, int r, int k>
        struct PinGroupElement
        {
            using Type = typename decltype(Vector<T, p, q, r>() * typename PinGroupElement<T, p, q, r, k - 1>::Type())::Type;

            static auto Random()
            {
                return Vector<T, p, q, r>::Random().normalized() * PinGroupElement<T, p, q, r, k - 1>::Random();
            };
        };

        template <class T, int p, int q, int r>
        struct PinGroupElement<T, p, q, r, 1>
        {
            using Type = Vector<T, p, q, r>;

            static auto Random()
            {
                return Vector<T, p, q, r>::Random().normalized();
            };
        };
    }  // namespace pin_detail

    template <class T, int p, int q, int r>
    class PinGroup : public Subgroup<T, p, q, r, CliffordGroup<T, p, q, r>>,  //
                     public CoveringGroup<T, p, q, r, OrthogonalGroup<T, p, q, r>>
    {
      public:
        constexpr static int dimension = Group<T, p, q, r>::dimension;
        using typename CliffordGroup<T, p, q, r>::Algebra;
        using Vector = typename CliffordGroup<T, p, q, r>::Vector;

      public:
        PinGroup() = default;

        virtual ~PinGroup() = default;

        template <int k>
            requires((k > 0) && (k <= dimension))
        using Element = typename pin_detail::PinGroupElement<T, p, q, r, k>::Type;

        template <int k>
        static Element<k> RandomElement();

      protected:
      private:
    };

}  // namespace gafro::groups

#include <gafro/groups/PinGroup.hxx>