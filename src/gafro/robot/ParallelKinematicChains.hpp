#pragma once

namespace gafro
{

    template <class T, int number>
    class ParallelKinematicChains
    {
      public:
        ParallelKinematicChains();

        virtual ~ParallelKinematicChains();

      protected:
      private:
        std::array<std::string, number> kinematic_chains_;
    };

}  // namespace gafro