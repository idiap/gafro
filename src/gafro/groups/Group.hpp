#pragma once

#include <concepts>

namespace gafro::groups
{

    template <class T, int p, int q, int r>
    class Group
    {
      public:
        constexpr static int dimension = p + q + r;

      public:
        Group() = default;

        virtual ~Group() = default;

      protected:
      private:
    };

    template <class H, class T, int p, int q, int r>
    concept DerivedGroup = std::is_base_of<Group<T, p, q, r>, H>::value;

}  // namespace gafro::groups