#pragma once

#include <gafro/groups/ExponentialMap.hpp>

namespace gafro::groups
{

    template <class Space, class Group, ExponentialMapOptions options>
    ExponentialMap<Space, Group, options>::ExponentialMap() = default;

    template <class Space, class Group, ExponentialMapOptions options>
    ExponentialMap<Space, Group, options>::~ExponentialMap() = default;

    template <class Space, class Group, ExponentialMapOptions options>
    constexpr bool ExponentialMap<Space, Group, options>::isSurjective()
    {
        return (options & Surjective) != 0;
    }

    template <class Space, class Group, ExponentialMapOptions options>
    constexpr bool ExponentialMap<Space, Group, options>::isWellDefined()
    {
        return (options & WellDefined) != 0;
    }

}  // namespace gafro::groups