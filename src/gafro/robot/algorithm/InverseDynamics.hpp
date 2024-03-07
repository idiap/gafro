#pragma once

#include <gafro/robot/System.hpp>

namespace gafro::algorithm
{

    template <class T, int dof>
    class InverseDynamics
    {
      public:
        Eigen::Vector<T, dof> compute(const KinematicChain<T> *kinematic_chain,   //
                                      const Eigen::Vector<T, dof> &position,      //
                                      const Eigen::Vector<T, dof> &velocity,      //
                                      const Eigen::Vector<T, dof> &acceleration,  //
                                      const T &gravity);

      private:
        template <class V, int j>
        struct ForwardIteration
        {
            static void compute(const std::vector<const Joint<T> *> &joints,           //
                                const std::vector<std::unique_ptr<Body<T>>> &bodies,   //
                                const Eigen::Vector<T, dof> &position,                 //
                                const Eigen::Vector<T, dof> &velocity,                 //
                                const Eigen::Vector<T, dof> &acceleration,             //
                                const Translator<T> &previous_com,                     //
                                std::array<Motor<T>, dof> &frame,                      //
                                std::array<Twist<T>, dof> &twist,                      //
                                std::array<Twist<T>, dof> &twist_dt,                   //
                                const E3i<T> &gravity);
        };

        template <class V, int j>
        struct BackwardIteration
        {
            static void compute(const std::vector<std::unique_ptr<Body<T>>> &bodies,   //
                                std::array<Motor<T>, dof> &frame,                      //
                                std::array<Twist<T>, dof> &twist,                      //
                                std::array<Twist<T>, dof> &twist_dt,                   //
                                Wrench<T> &wrench,                                     //
                                Eigen::Vector<T, dof> &torque);
        };

      private:
        std::array<Motor<T>, dof> frames_;

        std::array<Twist<T>, dof> twists_;

        std::array<Twist<T>, dof> twists_dt_;

        Wrench<T> wrench_;

        Eigen::Vector<T, dof> torque_;

        E3i<T> gravity_;
    };

}  // namespace gafro::algorithm