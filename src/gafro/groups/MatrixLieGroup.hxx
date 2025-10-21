#pragma once

#include <gafro/groups/MatrixLieGroup.hpp>

namespace gafro::groups
{

    template <class T, int p, int q, int r>
    MatrixLieGroup<T, p, q, r>::MatrixLieGroup() = default;

    template <class T, int p, int q, int r>
    MatrixLieGroup<T, p, q, r>::~MatrixLieGroup() = default;

}  // namespace gafro::groups