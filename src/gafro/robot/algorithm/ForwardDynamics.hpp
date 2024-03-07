#pragma once

#include <gafro/robot/System.hpp>

namespace gafro::algorithm
{

    namespace detail
    {
        template <class T, int dof, int j>
        class ForwardIteration
        {
          public:
            static void compute(const std::vector<std::unique_ptr<Joint<T>>> &joints,  //
                                const Eigen::Vector<T, dof> &position,                 //
                                const Eigen::Vector<T, dof> &velocity,                 //
                                const Eigen::Vector<T, dof> &acceleration,             //
                                const Translator<T> &previous_com,                     //
                                std::array<Motor<T>, dof> &frame,                      //
                                std::array<Twist<T>, dof> &twist,                      //
                                std::array<Twist<T>, dof> &twist_dt)
            {
                frame[j] = joints[j]->getChildLink()->getCenterOfMass().reverse() * joints[j]->getMotor(position[j]).reverse() * previous_com;

                twist[j] = twist[j - 1].transform(frame[j])  //
                           + Scalar<T>(velocity[j]) * joints[j]->getChildLink()->getAxis();

                twist_dt[j] = twist_dt[j - 1].transform(frame[j])                                                          //
                              + Scalar<T>(velocity[j]) * joints[j]->getChildLink()->getAxis().commutatorProduct(twist[j])  //
                              + Scalar<T>(acceleration[j]) * joints[j]->getChildLink()->getAxis();

                ForwardIteration<T, dof, j + 1>::compute(joints, position, velocity, acceleration, joints[j]->getChildLink()->getCenterOfMass(),
                                                         frame, twist, twist_dt);
            }
        };

        template <class T, int dof>
        class ForwardIteration<T, dof, 0>
        {
          public:
            static void compute(const std::vector<std::unique_ptr<Joint<T>>> &joints,  //
                                const Eigen::Vector<T, dof> &position,                 //
                                const Eigen::Vector<T, dof> &velocity,                 //
                                const Eigen::Vector<T, dof> &acceleration,             //
                                std::array<Motor<T>, dof> &frame,                      //
                                std::array<Twist<T>, dof> &twist,                      //
                                std::array<Twist<T>, dof> &twist_dt)
            {
                frame[0] = joints[0]->getChildLink()->getCenterOfMass().reverse() * joints[0]->getMotor(position[0]).reverse();

                twist[0] = Scalar<T>(velocity[0]) * joints[0]->getChildLink()->getAxis();

                twist_dt[0] = E3i<T>(9.81)                                                                                 //
                              + Scalar<T>(velocity[0]) * joints[0]->getChildLink()->getAxis().commutatorProduct(twist[0])  //
                              + Scalar<T>(acceleration[0]) * joints[0]->getChildLink()->getAxis();

                ForwardIteration<T, dof, 1>::compute(joints, position, velocity, acceleration, joints[0]->getChildLink()->getCenterOfMass(), frame,
                                                     twist, twist_dt);
            }
        };

        template <class T, int dof>
        class ForwardIteration<T, dof, dof>
        {
          public:
            static void compute(const std::vector<std::unique_ptr<Joint<T>>> &joints,  //
                                const Eigen::Vector<T, dof> &position,                 //
                                const Eigen::Vector<T, dof> &velocity,                 //
                                const Eigen::Vector<T, dof> &acceleration,             //
                                const Translator<T> &previous_com,                     //
                                std::array<Motor<T>, dof> &frame,                      //
                                std::array<Twist<T>, dof> &twist,                      //
                                std::array<Twist<T>, dof> &twist_dt)
            {}
        };

        template <class T, int dof, int j>
        class BackwardIteration
        {
          public:
            static void compute(const std::vector<std::unique_ptr<Joint<T>>> &joints,  //
                                std::array<Motor<T>, dof> &frame,                      //
                                std::array<Twist<T>, dof> &twist,                      //
                                std::array<Twist<T>, dof> &twist_dt,                   //
                                Wrench<T> &wrench,                                     //
                                Eigen::Vector<T, dof> &torque)
            {
                const Link<T> *link = joints[j]->getChildLink();

                wrench = frame[j + 1].reverse() * wrench * frame[j + 1]  //
                         + link->getInertia()(twist_dt[j])               //
                         - twist[j].commute(link->getInertia()(twist[j]));

                torque[j] = -(wrench | link->getAxis()).template get<blades::scalar>();

                BackwardIteration<T, dof, j - 1>::compute(joints, frame, twist, twist_dt, wrench, torque);
            }
        };

        template <class T, int dof>
        class BackwardIteration<T, dof, dof - 1>
        {
          public:
            static void compute(const std::vector<std::unique_ptr<Joint<T>>> &joints,  //
                                std::array<Motor<T>, dof> &frame,                      //
                                std::array<Twist<T>, dof> &twist,                      //
                                std::array<Twist<T>, dof> &twist_dt,                   //
                                Wrench<T> &wrench,                                     //
                                Eigen::Vector<T, dof> &torque)
            {
                const Link<T> *link = joints[dof - 1]->getChildLink();

                wrench = link->getInertia()(twist_dt[dof - 1])  //
                         - twist[dof - 1].commute(link->getInertia()(twist[dof - 1]));

                torque[dof - 1] = -(wrench | link->getAxis()).template get<blades::scalar>();

                BackwardIteration<T, dof, dof - 2>::compute(joints, frame, twist, twist_dt, wrench, torque);
            }
        };

        template <class T, int dof>
        class BackwardIteration<T, dof, -1>
        {
          public:
            static void compute(const std::vector<std::unique_ptr<Joint<T>>> &joints,  //
                                std::array<Motor<T>, dof> &frame,                      //
                                std::array<Twist<T>, dof> &twist,                      //
                                std::array<Twist<T>, dof> &twist_dt,                   //
                                Wrench<T> &wrench,                                     //
                                Eigen::Vector<T, dof> &torque)
            {}
        };
    }  // namespace detail

    template <class T, int dof>
    class ForwardDynamics
    {
      public:
        ForwardDynamics();

        virtual ~ForwardDynamics();

        static Eigen::Vector<T, dof> compute(const System<T> &system,                //
                                             const Eigen::Vector<T, dof> &position,  //
                                             const Eigen::Vector<T, dof> &velocity,  //
                                             const Eigen::Vector<T, dof> &acceleration)
        {
            std::array<Motor<T>, dof> frames;
            std::array<Twist<T>, dof> twists;
            std::array<Twist<T>, dof> twists_dt;

            detail::ForwardIteration<T, dof, 0>::compute(system.getJoints(), position, velocity, acceleration, frames, twists, twists_dt);

            Wrench<T> wrench(Wrench<T>::Parameters::Zero());
            Eigen::Vector<T, dof> torque;

            detail::BackwardIteration<T, dof, dof - 1>::compute(system.getJoints(), frames, twists, twists_dt, wrench, torque);

            return torque;
        }

      private:
    };

}  // namespace gafro::algorithm