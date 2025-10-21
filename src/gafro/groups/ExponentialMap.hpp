#pragma once

namespace gafro::groups
{
    enum ExponentialMapOptions
    {
        Surjective = 0x01,
        WellDefined = 0x02
    };

    template <class Space, class Group, ExponentialMapOptions options>
    class ExponentialMap
    {
      public:
        ExponentialMap();

        virtual ~ExponentialMap();

        constexpr static bool isSurjective();

        constexpr static bool isWellDefined();

      protected:
      private:
    };

}  // namespace gafro::groups