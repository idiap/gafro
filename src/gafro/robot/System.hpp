// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/gafro_package_config.hpp>
//
#include <gafro/robot/Joint.hpp>
#include <gafro/robot/KinematicChain.hpp>
#include <gafro/robot/Link.hpp>

namespace gafro
{
    template <class T>
    class ForwardKinematics;

    template <class T, int size, int dof>
    class CooperativeTaskSpace;

    template <class T>
    class System
    {
      public:
        System();

        System(const System &other) = delete;

        System(System &&other);

        System &operator=(const System &other) = delete;

        System &operator=(System &&other);

        friend class SystemSerialization;

      public:
        virtual ~System();

        void add(const std::string &parent_link, std::unique_ptr<Joint<T>> &&joint, const System &system, const std::string &pre = "");

        void addJoint(std::unique_ptr<Joint<T>> &&joint);

        void addLink(std::unique_ptr<Link<T>> &&link);

        void addKinematicChain(const std::string &name, std::unique_ptr<KinematicChain<T>> &&kinematic_chain);

        void finalize();

        void setName(const std::string &name);

        const int &getDoF() const;

        const Link<T> &getBaseLink() const;

        Link<T> *getLink(const std::string &name);

        const Link<T> *getLink(const std::string &name) const;

        std::vector<std::unique_ptr<Link<T>>> &getLinks();

        const std::vector<std::unique_ptr<Link<T>>> &getLinks() const;

        Joint<T> *getJoint(const std::string &name);

        const Joint<T> *getJoint(const std::string &name) const;

        std::vector<std::unique_ptr<Joint<T>>> &getJoints();

        const std::vector<std::unique_ptr<Joint<T>>> &getJoints() const;

        template <class S>
        System<S> cast() const;

        System copy() const;

        bool hasKinematicChain(const std::string &name) const;

        KinematicChain<T> *getKinematicChain(const std::string &name);

        const KinematicChain<T> *getKinematicChain(const std::string &name) const;

        const std::vector<std::unique_ptr<KinematicChain<T>>> &getKinematicChains() const;

        const std::string &getName() const;

        //

        Motor<T> computeLinkMotor(const std::string &name, const Eigen::Vector<T, Eigen::Dynamic> &position) const;

        ForwardKinematics<T> computeForwardKinematics(const Eigen::VectorX<T> &joint_positions, const Motor<T> &base_motor = Motor<T>()) const;

        //

        template <int dof>
        Motor<T> computeKinematicChainMotor(const std::string &name, const Eigen::Vector<T, dof> &position) const;

        template <int dof>
        MultivectorMatrix<T, Motor, 1, dof> computeKinematicChainAnalyticJacobian(const std::string           &name,
                                                                                  const Eigen::Vector<T, dof> &position) const;

        template <int dof>
        MultivectorMatrix<T, MotorGenerator, 1, dof> computeKinematicChainGeometricJacobian(const std::string           &name,
                                                                                            const Eigen::Vector<T, dof> &position) const;

        template <int dof>
        MultivectorMatrix<T, MotorGenerator, 1, dof> computeKinematicChainGeometricJacobianBody(const std::string           &name,
                                                                                                const Eigen::Vector<T, dof> &position) const;

        template <int dof>
        MultivectorMatrix<T, MotorGenerator, 1, dof> computeKinematicChainGeometricJacobianTimeDerivative(const std::string           &name,
                                                                                                          const Eigen::Vector<T, dof> &position,
                                                                                                          const Eigen::Vector<T, dof> &velocity,
                                                                                                          const Motor<T> &reference) const;

        template <int dof>
        Eigen::Vector<T, dof> computeInverseDynamics(const Eigen::Vector<T, dof> &position,
                                                     const Eigen::Vector<T, dof> &velocity,
                                                     const Eigen::Vector<T, dof> &acceleration,
                                                     const T                     &gravity              = 9.81,
                                                     const Wrench<T>              ee_wrench            = Wrench<T>::Zero(),
                                                     const std::string           &kinematic_chain_name = "") const;

        template <int dof>
        Eigen::Vector<T, dof> computeForwardDynamics(const Eigen::Vector<T, dof> &position,
                                                     const Eigen::Vector<T, dof> &velocity,
                                                     const Eigen::Vector<T, dof> &torque,
                                                     const std::string           &kinematic_chain_name = "") const;

        template <int dof>
        Eigen::Matrix<T, dof, dof> computeKinematicChainMassMatrix(const std::string &name, const Eigen::Vector<T, dof> &position) const;

        std::vector<const Joint<T> *> getJointChain(const std::string &name) const;

        template <int size, int dof>
        CooperativeTaskSpace<T, size, dof> createCooperativeTaskSpace(const std::array<std::string, size> &kinematic_chains) ;

        void createKinematicChain(const std::string &joint_name);

      public:
        using Vector = Eigen::VectorX<T>;

        Vector getRandomConfiguration() const;

        void setJointLimits(const Vector &joint_limits_min, const Vector &joint_limits_max);

        const Vector &getJointLimitsMin() const;

        const Vector &getJointLimitsMax() const;

        bool isJointPositionFeasible(const Vector &position) const;

      private:
        void createJointLimits();

      private:
        std::vector<std::unique_ptr<Link<T>>> links_;

        std::vector<std::unique_ptr<Joint<T>>> joints_;

        std::vector<std::unique_ptr<KinematicChain<T>>> kinematic_chains_;

        std::map<std::string, Joint<T> *> joints_map_;

        std::map<std::string, Link<T> *> links_map_;

        std::map<std::string, KinematicChain<T> *> kinematic_chains_map_;

        std::string name_;

        int dof_ = 0;

        Vector joint_limits_min_;

        Vector joint_limits_max_;

      public:
        // static System<T> load(const std::string &filename);
    };

}  // namespace gafro

#include <gafro/robot/System.hxx>
