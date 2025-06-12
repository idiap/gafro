// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/Body.hxx>
#include <gafro/robot/Joint.hxx>

namespace gafro
{

    template <class T>
    class KinematicChain
    {
      public:
        KinematicChain(const std::string &name = "");

        KinematicChain(const KinematicChain &other) = delete;

        KinematicChain(KinematicChain &&other);

        virtual ~KinematicChain();

        KinematicChain &operator=(const KinematicChain &other) = delete;

        KinematicChain &operator=(KinematicChain &&other);

        int getDoF() const;

        const std::string &getName() const;

        void addActuatedJoint(const Joint<T> *joint);

        void addFixedMotor(const Motor<T> &motor);

        void setFixedMotors(const std::map<int, Motor<T>> &fixed_motors);

        const std::map<int, Motor<T>> &getFixedMotors() const;

        const std::vector<const Joint<T> *> &getActuatedJoints() const;

        const std::vector<std::unique_ptr<Body<T>>> &getBodies() const;

        template <int dof>
        Motor<T> computeMotor(const Eigen::Vector<T, dof> &position) const;

        Motor<T> computeMotor(const int &index, const T &position) const;

        Motor<T> computeMotorDerivative(const int &index, const T &position) const;

        template <int dof>
        MultivectorMatrix<T, Motor, 1, dof> computeAnalyticJacobian(const Eigen::Vector<T, dof> &position) const;

        template <int dof>
        MultivectorMatrix<T, MotorGenerator, 1, dof> computeGeometricJacobian(const Eigen::Vector<T, dof> &position) const;

        template <int dof>
        MultivectorMatrix<T, MotorGenerator, 1, dof> computeGeometricJacobianBody(const Eigen::Vector<T, dof> &position) const;

        template <int dof>
        MultivectorMatrix<T, MotorGenerator, 1, dof> computeKinematicChainGeometricJacobianTimeDerivative(const Eigen::Vector<T, dof> &position,
                                                                                                          const Eigen::Vector<T, dof> &velocity,
                                                                                                          const Motor<T> &reference) const;

        template <int dof>
        Eigen::Matrix<T, dof, dof> computeMassMatrix(const Eigen::Vector<T, dof> &position) const;

        void finalize();

      protected:
        void createBody(const Joint<T> *startJoint);

      private:
        std::vector<const Joint<T> *> actuated_joints_;

        std::map<int, Motor<T>> fixed_motors_;

        std::vector<std::unique_ptr<Body<T>>> bodies_;

        std::string name_;
    };

}  // namespace gafro

#include <gafro/robot/KinematicChain.hxx>