#pragma once

#include <gafro/robot/algorithm/InverseDynamics.hpp>

namespace gafro::algorithm
{

    template <class T, int dof>
    Eigen::Vector<T, dof> InverseDynamics<T, dof>::compute(const KinematicChain<T> *kinematic_chain,   //
                                                           const Eigen::Vector<T, dof> &position,      //
                                                           const Eigen::Vector<T, dof> &velocity,      //
                                                           const Eigen::Vector<T, dof> &acceleration,  //
                                                           const T &gravity)
    {
        if (dof != kinematic_chain->getDoF())
        {
            throw std::runtime_error("Incorrect number of DOF");
        }

        gravity_ = E3i<T>(gravity);

        ForwardIteration<T, 0>::compute(
            kinematic_chain->getActuatedJoints(),
            kinematic_chain->getBodies(),
            position,
            velocity,
            acceleration,
            frames_,
            twists_,
            twists_dt_,
            gravity_
        );

        BackwardIteration<T, 0>::compute(
            kinematic_chain->getBodies(),
            frames_,
            twists_,
            twists_dt_,
            wrench_,
            torque_
        );

        return torque_;
    }

    template <class T, int dof>
    template <class V, int j>
    void InverseDynamics<T, dof>::ForwardIteration<V, j>::compute(const std::vector<const Joint<T> *> &joints,           //
                                                                  const std::vector<std::unique_ptr<Body<T>>> &bodies,   //
                                                                  const Eigen::Vector<T, dof> &position,                 //
                                                                  const Eigen::Vector<T, dof> &velocity,                 //
                                                                  const Eigen::Vector<T, dof> &acceleration,             //
                                                                  const Translator<T> &previous_com,                     //
                                                                  std::array<Motor<T>, dof> &frame,                      //
                                                                  std::array<Twist<T>, dof> &twist,                      //
                                                                  std::array<Twist<T>, dof> &twist_dt,                   //
                                                                  const E3i<T> &gravity)
    {
        frame[j] = bodies[j]->getCenterOfMass().reverse() * joints[j]->getMotor(position[j]).reverse() * previous_com;

        twist[j] = twist[j - 1].transform(frame[j])  //
                   + Scalar<T>(velocity[j]) * bodies[j]->getAxis();

        twist_dt[j] = twist_dt[j - 1].transform(frame[j])                                                          //
                      + Scalar<T>(velocity[j]) * bodies[j]->getAxis().commutatorProduct(twist[j])  //
                      + Scalar<T>(acceleration[j]) * bodies[j]->getAxis();

        ForwardIteration<V, j + 1>::compute(joints, bodies, position, velocity, acceleration, bodies[j]->getCenterOfMass(), frame, twist,
                                            twist_dt, gravity);
    }

    template <class T, int dof>
    template <class V>
    struct InverseDynamics<T, dof>::ForwardIteration<V, 0>
    {
        static void compute(const std::vector<const Joint<T> *> &joints,           //
                            const std::vector<std::unique_ptr<Body<T>>> &bodies,   //
                            const Eigen::Vector<T, dof> &position,                 //
                            const Eigen::Vector<T, dof> &velocity,                 //
                            const Eigen::Vector<T, dof> &acceleration,             //
                            std::array<Motor<T>, dof> &frame,                      //
                            std::array<Twist<T>, dof> &twist,                      //
                            std::array<Twist<T>, dof> &twist_dt,                   //
                            const E3i<T> &gravity)
        {
            frame[0] = bodies[0]->getCenterOfMass().reverse() * joints[0]->getMotor(position[0]).reverse();

            twist[0] = Scalar<T>(velocity[0]) * bodies[0]->getAxis();

            twist_dt[0] = gravity                                                                                      //
                          + Scalar<T>(velocity[0]) * bodies[0]->getAxis().commutatorProduct(twist[0])  //
                          + Scalar<T>(acceleration[0]) * bodies[0]->getAxis();

            ForwardIteration<V, 1>::compute(joints, bodies, position, velocity, acceleration, bodies[0]->getCenterOfMass(), frame, twist,
                                            twist_dt, gravity);
        }
    };

    // end the forward recursion
    template <class T, int dof>
    template <class V>
    struct InverseDynamics<T, dof>::ForwardIteration<V, dof>
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
                            const E3i<T> &gravity)
        {}
    };

    template <class T, int dof>
    template <class V, int j>
    void InverseDynamics<T, dof>::BackwardIteration<V, j>::compute(const std::vector<std::unique_ptr<Body<T>>> &bodies,   //
                                                                   std::array<Motor<T>, dof> &frame,                      //
                                                                   std::array<Twist<T>, dof> &twist,                      //
                                                                   std::array<Twist<T>, dof> &twist_dt,                   //
                                                                   Wrench<T> &wrench,                                     //
                                                                   Eigen::Vector<T, dof> &torque)
    {
        // index needs to be computed starting from the back, this implementation is necessary due to gcc issues when specializing a template
        // using dof-1
        constexpr int k = dof - 1 - j;

        wrench = frame[k + 1].reverse() * wrench * frame[k + 1]  //
                 + bodies[k]->getInertia()(twist_dt[k])               //
                 - twist[k].commute(bodies[k]->getInertia()(twist[k]));

        torque[k] = -(wrench | bodies[k]->getAxis()).template get<blades::scalar>();

        BackwardIteration<V, j + 1>::compute(bodies, frame, twist, twist_dt, wrench, torque);
    }

    template <class T, int dof>
    template <class V>
    struct InverseDynamics<T, dof>::BackwardIteration<V, 0>
    {
        static void compute(const std::vector<std::unique_ptr<Body<T>>> &bodies,   //
                            std::array<Motor<T>, dof> &frame,                      //
                            std::array<Twist<T>, dof> &twist,                      //
                            std::array<Twist<T>, dof> &twist_dt,                   //
                            Wrench<T> &wrench,                                     //
                            Eigen::Vector<T, dof> &torque)
        {
            wrench = bodies[dof - 1]->getInertia()(twist_dt[dof - 1])  //
                     - twist[dof - 1].commute(bodies[dof - 1]->getInertia()(twist[dof - 1]));

            torque[dof - 1] = -(wrench | bodies[dof - 1]->getAxis()).template get<blades::scalar>();

            BackwardIteration<V, 1>::compute(bodies, frame, twist, twist_dt, wrench, torque);
        }
    };

    // end the backward recursion
    template <class T, int dof>
    template <class V>
    struct InverseDynamics<T, dof>::BackwardIteration<V, dof>
    {
        static void compute(const std::vector<std::unique_ptr<Body<T>>> &bodies,   //
                            std::array<Motor<T>, dof> &frame,                      //
                            std::array<Twist<T>, dof> &twist,                      //
                            std::array<Twist<T>, dof> &twist_dt,                   //
                            Wrench<T> &wrench,                                     //
                            Eigen::Vector<T, dof> &torque)
        {}
    };

}  // namespace gafro::algorithm