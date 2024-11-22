#pragma once

#include <gafro/groups/PinGroup.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r>
    template <int k>
    PinGroup<T, p, q, r>::Element<k> PinGroup<T, p, q, r>::RandomElement()
    {
        return pin_detail::PinGroupElement<T, p, q, r, k>::Random();
    }

}  // namespace gafro::groups