// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/DualManipulator.hpp>
#include <gafro/robot/FixedJoint.hpp>
#include <gafro/robot/PrismaticJoint.hpp>
#include <gafro/robot/RevoluteJoint.hpp>

namespace gafro
{

    template <class T, int dof>
    DualManipulator<T, dof>::DualManipulator(System<T> &&system, const std::string &first_ee_joint_name, const std::string &second_ee_joint_name)
      : System<T>(std::move(system)), first_ee_joint_name_(first_ee_joint_name), second_ee_joint_name_(second_ee_joint_name)
    {
        this->createKinematicChain(first_ee_joint_name);
        this->createKinematicChain(second_ee_joint_name);

        assert(this->getLink("base_link"));
    }

    template <class T, int dof>
    DualManipulator<T, dof>::DualManipulator(const Manipulator<T, dof / 2> &manipulator, const Motor<T> &first_base_motor,
                                             const Motor<T> &second_base_motor)
    {
        std::unique_ptr<Link<T>> base_link = std::make_unique<Link<T>>();
        base_link->setName("base_link");

        std::unique_ptr<FixedJoint<T>> first_base_joint = std::make_unique<FixedJoint<T>>();
        first_base_joint->setName("first_base_joint");
        first_base_joint->setFrame(first_base_motor);

        std::unique_ptr<FixedJoint<T>> second_base_joint = std::make_unique<FixedJoint<T>>();
        second_base_joint->setName("second_base_joint");
        second_base_joint->setFrame(second_base_motor);

        first_base_joint->setParentLink(base_link.get());
        base_link->addChildJoint(first_base_joint.get());

        second_base_joint->setParentLink(base_link.get());
        base_link->addChildJoint(second_base_joint.get());

        this->addLink(std::move(base_link));
        this->addJoint(std::move(first_base_joint));
        this->addJoint(std::move(second_base_joint));

        const auto &links = manipulator.getSystem().getLinks();
        const auto &joints = manipulator.getSystem().getJoints();

        for (const std::string pre : { "first_", "second_" })
        {
            for (unsigned j = 0; j < links.size(); ++j)
            {
                std::unique_ptr<Link<T>> link = std::make_unique<Link<T>>();

                link->setName(pre + links[j]->getName());
                link->setCenterOfMass(links[j]->getCenterOfMass());
                link->setInertia(links[j]->getInertia());
                link->setMass(links[j]->getMass());
                link->setAxis(links[j]->getAxis());

                this->addLink(std::move(link));
            }

            for (unsigned j = 0; j < joints.size(); ++j)
            {
                std::unique_ptr<Joint<T>> joint;

                switch (joints[j]->getType())
                {
                case Joint<T>::Type::FIXED: {
                    joint = std::make_unique<FixedJoint<T>>();

                    break;
                }
                case Joint<T>::Type::REVOLUTE: {
                    joint = std::make_unique<RevoluteJoint<T>>();

                    static_cast<RevoluteJoint<T> *>(joint.get())->setAxis(static_cast<RevoluteJoint<T> *>(joints[j].get())->getAxis());

                    break;
                }
                case Joint<T>::Type::PRISMATIC: {
                    joint = std::make_unique<PrismaticJoint<T>>();

                    static_cast<PrismaticJoint<T> *>(joint.get())->setAxis(static_cast<PrismaticJoint<T> *>(joints[j].get())->getAxis());

                    break;
                }
                }

                joint->setName(pre + joints[j]->getName());
                joint->setFrame(joints[j]->getFrame());
                joint->setLimits(joints[j]->getLimits());

                this->addJoint(std::move(joint));
            }

            for (const std::unique_ptr<Link<T>> &link : links)
            {
                {
                    auto child_joints = manipulator.getLink(link->getName())->getChildJoints();

                    for (const auto &joint : child_joints)
                    {
                        this->getLink(pre + link->getName())->addChildJoint(this->getJoint(pre + joint->getName()));
                    }
                }

                {
                    auto parent_joint = manipulator.getLink(link->getName())->getParentJoint();

                    if (parent_joint)
                    {
                        this->getLink(pre + link->getName())->setParentJoint(this->getJoint(pre + parent_joint->getName()));
                    }
                }
            }

            for (const std::unique_ptr<Joint<T>> &joint : joints)
            {
                {
                    auto link = joint->getChildLink();

                    if (link)
                    {
                        this->getJoint(pre + joint->getName())->setChildLink(this->getLink(pre + link->getName()));
                    }
                }

                {
                    auto link = joint->getParentLink();

                    if (link)
                    {
                        this->getJoint(pre + joint->getName())->setParentLink(this->getLink(pre + link->getName()));
                    }
                }
            }
        }

        this->getLink("first_" + links.front()->getName())->setParentJoint(this->getJoint("first_base_joint"));
        this->getLink("second_" + links.front()->getName())->setParentJoint(this->getJoint("second_base_joint"));

        this->getJoint("first_base_joint")->setChildLink(this->getLink("first_" + links.front()->getName()));
        this->getJoint("second_base_joint")->setChildLink(this->getLink("second_" + links.front()->getName()));

        this->finalize();

        first_ee_joint_name_ = "first_" + manipulator.getEEKinematicChain()->getName();
        second_ee_joint_name_ = "second_" + manipulator.getEEKinematicChain()->getName();
    }

    template <class T, int dof>
    DualManipulator<T, dof>::DualManipulator(DualManipulator &&manipulator)
    {
        *this = std::move(manipulator);
    }

    template <class T, int dof>
    DualManipulator<T, dof> &DualManipulator<T, dof>::operator=(DualManipulator &&manipulator)
    {
        this->System<T>::operator=(std::move(manipulator));
        first_ee_joint_name_ = std::move(manipulator.first_ee_joint_name_);
        second_ee_joint_name_ = std::move(manipulator.second_ee_joint_name_);

        return *this;
    }

    template <class T, int dof>
    DualManipulator<T, dof>::~DualManipulator() = default;

    //

    template <class T, int dof>
    const System<T> &DualManipulator<T, dof>::getSystem() const
    {
        return *this;
    }

    //

    template <class T, int dof>
    Motor<T> DualManipulator<T, dof>::getFirstBaseMotor() const
    {
        return this->getLink("base_link")->getChildJoints()[0]->getMotor(0.0);
    }

    template <class T, int dof>
    Motor<T> DualManipulator<T, dof>::getSecondBaseMotor() const
    {
        return this->getLink("base_link")->getChildJoints()[1]->getMotor(0.0);
    }

    //

    template <class T, int dof>
    Motor<T> DualManipulator<T, dof>::getFirstEEMotor(const Eigen::Vector<T, dof / 2> &position) const
    {
        return this->computeKinematicChainMotor(first_ee_joint_name_, position);
    }

    template <class T, int dof>
    Motor<T> DualManipulator<T, dof>::getSecondEEMotor(const Eigen::Vector<T, dof / 2> &position) const
    {
        return this->computeKinematicChainMotor(second_ee_joint_name_, position);
    }

    template <class T, int dof>
    const KinematicChain<T> *DualManipulator<T, dof>::getFirstKinematicChain() const
    {
        return this->getKinematicChain(first_ee_joint_name_);
    }

    template <class T, int dof>
    const KinematicChain<T> *DualManipulator<T, dof>::getSecondKinematicChain() const
    {
        return this->getKinematicChain(second_ee_joint_name_);
    }

    template <class T, int dof>
    Motor<T> DualManipulator<T, dof>::getAbsoluteMotor(const Eigen::Vector<T, dof> &positions) const
    {
        return this->getAbsoluteMotor(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Motor<T> DualManipulator<T, dof>::getAbsoluteMotor(const Eigen::Vector<T, dof / 2> &position_first,
                                                       const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return getSecondEEMotor(position_second) *
               Motor<T>::exp(Scalar<T>(TypeTraits<T>::Value(0.5)) * getRelativeMotor(position_first, position_second).log());
    }

    template <class T, int dof>
    Motor<T> DualManipulator<T, dof>::getRelativeMotor(const Eigen::Vector<T, dof> &positions) const
    {
        return this->getRelativeMotor(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Motor<T> DualManipulator<T, dof>::getRelativeMotor(const Eigen::Vector<T, dof / 2> &position_first,
                                                       const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return getSecondEEMotor(position_second).reverse() * getFirstEEMotor(position_first);
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, dof> DualManipulator<T, dof>::getRelativeAnalyticJacobian(const Eigen::Vector<T, dof> &positions) const
    {
        return getRelativeAnalyticJacobian(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, dof> DualManipulator<T, dof>::getRelativeAnalyticJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                             const Eigen::Vector<T, dof / 2> &position_second) const
    {
        Motor<T> first_motor = getFirstEEMotor(position_first);
        Motor<T> second_motor = getSecondEEMotor(position_second);
        MultivectorMatrix<T, Motor, 1, dof / 2> first_jacobian = this->computeKinematicChainAnalyticJacobian(first_ee_joint_name_, position_first);
        MultivectorMatrix<T, Motor, 1, dof / 2> second_jacobian = this->computeKinematicChainAnalyticJacobian(second_ee_joint_name_, position_second);

        MultivectorMatrix<T, Motor, 1, dof> jacobian;

        for (unsigned j = 0; j < dof / 2; ++j)
        {
            jacobian.setCoefficient(0, j, second_motor.reverse() * first_jacobian.getCoefficient(0, j));
            jacobian.setCoefficient(0, j + dof / 2, Motor<T>(second_jacobian.getCoefficient(0, j)).reverse() * first_motor);
        }

        return jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> DualManipulator<T, dof>::getRelativeGeometricJacobian(const Eigen::Vector<T, dof> &positions,
                                                                                                       const Motor<T> &reference) const
    {
        return getRelativeGeometricJacobian(positions.topRows(dof / 2), positions.bottomRows(dof / 2), reference);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> DualManipulator<T, dof>::getRelativeGeometricJacobian(
      const Eigen::Vector<T, dof / 2> &position_first, const Eigen::Vector<T, dof / 2> &position_second, const Motor<T> &reference) const
    {
        Motor<T> relative_motor = getRelativeMotor(position_first, position_second);
        MultivectorMatrix<T, Motor, 1, dof> relative_analytic_jacobian = getRelativeAnalyticJacobian(position_first, position_second);
        MultivectorMatrix<T, MotorGenerator, 1, dof> relative_geometric_jacobian;

        for (int j = 0; j < dof; ++j)
        {
            relative_geometric_jacobian.setCoefficient(
              0, j,
              reference.reverse() *
                (Scalar<T>(TypeTraits<T>::Value(-2.0)) * relative_analytic_jacobian.getCoefficient(0, j) * relative_motor.reverse()) * reference);
        }

        return relative_geometric_jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> DualManipulator<T, dof>::getAbsoluteGeometricJacobian(const Eigen::Vector<T, dof> &positions,
                                                                                                       const Motor<T> &reference) const
    {
        return getAbsoluteGeometricJacobian(positions.topRows(dof / 2), positions.bottomRows(dof / 2), reference);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> DualManipulator<T, dof>::getAbsoluteGeometricJacobian(
      const Eigen::Vector<T, dof / 2> &position_first, const Eigen::Vector<T, dof / 2> &position_second, const Motor<T> &reference) const
    {
        Motor<T> absolute_motor = getAbsoluteMotor(position_first, position_second);
        MultivectorMatrix<T, Motor, 1, dof> absolute_analytic_jacobian = getAbsoluteAnalyticJacobian(position_first, position_second);

        MultivectorMatrix<T, MotorGenerator, 1, dof> absolute_geometric_jacobian;

        for (int j = 0; j < dof; ++j)
        {
            absolute_geometric_jacobian.setCoefficient(
              0, j,
              reference.reverse() *
                (Scalar<T>(TypeTraits<T>::Value(-2.0)) * absolute_analytic_jacobian.getCoefficient(0, j) * absolute_motor.reverse()) * reference);
        }

        return absolute_geometric_jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> DualManipulator<T, dof>::getRelativeGeometricJacobianTimeDerivative(
      const Eigen::Vector<T, dof> &positions, const Eigen::Vector<T, dof> &velocities, const Motor<T> &reference) const
    {
        return getRelativeGeometricJacobianTimeDerivative(positions.topRows(dof / 2), positions.bottomRows(dof / 2), velocities.topRows(dof / 2),
                                                          velocities.bottomRows(dof / 2), reference);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> DualManipulator<T, dof>::getRelativeGeometricJacobianTimeDerivative(
      const Eigen::Vector<T, dof / 2> &position_first, const Eigen::Vector<T, dof / 2> &position_second,
      const Eigen::Vector<T, dof / 2> &velocity_first, const Eigen::Vector<T, dof / 2> &velocity_second, const Motor<T> &reference) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, dof> relative_geometric_jacobian = getRelativeGeometricJacobian(position_first, position_second);
        Eigen::Matrix<T, 6, dof> relative_geometric_jacobian_matrix = relative_geometric_jacobian.embed();
        MultivectorMatrix<T, MotorGenerator, 1, dof> relative_geometric_jacobian_time_derivative;

        for (int j = 0; j < dof / 2; ++j)
        {
            relative_geometric_jacobian_time_derivative.setCoefficient(
              0, j,
              reference.reverse() *
                (relative_geometric_jacobian.getCoefficient(0, j).commute(
                  Twist<T>(relative_geometric_jacobian_matrix.block(0, 0, 6, j + 1) * velocity_first.topRows(j + 1)))) *
                reference);

            relative_geometric_jacobian_time_derivative.setCoefficient(
              0, dof / 2 + j,
              reference.reverse() *
                (relative_geometric_jacobian.getCoefficient(0, dof / 2 + j)
                   .commute(Twist<T>(relative_geometric_jacobian_matrix.block(0, dof / 2, 6, j + 1) * velocity_second.topRows(j + 1)))) *
                reference);
        }

        return relative_geometric_jacobian_time_derivative;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> DualManipulator<T, dof>::getAbsoluteGeometricJacobianTimeDerivative(
      const Eigen::Vector<T, dof> &positions, const Eigen::Vector<T, dof> &velocities, const Motor<T> &reference) const
    {
        return getAbsoluteGeometricJacobianTimeDerivative(positions.topRows(dof / 2), positions.bottomRows(dof / 2), velocities.topRows(dof / 2),
                                                          velocities.bottomRows(dof / 2), reference);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> DualManipulator<T, dof>::getAbsoluteGeometricJacobianTimeDerivative(
      const Eigen::Vector<T, dof / 2> &position_first, const Eigen::Vector<T, dof / 2> &position_second,
      const Eigen::Vector<T, dof / 2> &velocity_first, const Eigen::Vector<T, dof / 2> &velocity_second, const Motor<T> &reference) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, dof> absolute_geometric_jacobian = getAbsoluteGeometricJacobian(position_first, position_second);
        Eigen::Matrix<T, 6, dof> absolute_geometric_jacobian_matrix = absolute_geometric_jacobian.embed();
        MultivectorMatrix<T, MotorGenerator, 1, dof> absolute_geometric_jacobian_time_derivative;

        for (int j = 0; j < dof / 2; ++j)
        {
            absolute_geometric_jacobian_time_derivative.setCoefficient(
              0, j,
              reference.reverse() *
                (absolute_geometric_jacobian.getCoefficient(0, j).commute(
                  Twist<T>(absolute_geometric_jacobian_matrix.block(0, 0, 6, j + 1) * velocity_first.topRows(j + 1)))) *
                reference);

            absolute_geometric_jacobian_time_derivative.setCoefficient(
              0, dof / 2 + j,
              reference.reverse() *
                (absolute_geometric_jacobian.getCoefficient(0, dof / 2 + j)
                   .commute(Twist<T>(absolute_geometric_jacobian_matrix.block(0, dof / 2, 6, j + 1) * velocity_second.topRows(j + 1)))) *
                reference);
        }

        return absolute_geometric_jacobian_time_derivative;
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, dof> DualManipulator<T, dof>::getAbsoluteAnalyticJacobian(const Eigen::Vector<T, dof> &positions) const
    {
        return getAbsoluteAnalyticJacobian(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, dof> DualManipulator<T, dof>::getAbsoluteAnalyticJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                             const Eigen::Vector<T, dof / 2> &position_second) const
    {
        Motor<T> relative_motor = getRelativeMotor(position_first, position_second);
        typename Motor<T>::Generator b = Scalar<T>(TypeTraits<T>::Value(0.5)) * relative_motor.log();
        Motor<T> motor = Motor<T>::exp(b);
        Motor<T> second_motor = getSecondEEMotor(position_second);

        MultivectorMatrix<T, Motor, 1, dof / 2> second_jacobian = this->computeKinematicChainAnalyticJacobian(second_ee_joint_name_, position_second);

        MultivectorMatrix<T, Motor, 1, dof> absolute_jacobian;

        Eigen::Matrix<T, 8, 6> exp_jacobian = Motor<T>::Exponential::getJacobian(b);
        Eigen::Matrix<T, 6, 8> log_jacobian = Motor<T>::Logarithm::getJacobian(relative_motor);
        Eigen::Matrix<T, 8, dof> relative_analytic_jacobian = getRelativeAnalyticJacobian(position_first, position_second).embed();

        Eigen::Matrix<T, 8, dof> j = TypeTraits<T>::Value(0.5) * exp_jacobian * log_jacobian * relative_analytic_jacobian;

        for (unsigned i = 0; i < dof / 2; ++i)
        {
            absolute_jacobian.setCoefficient(0, i, second_motor * Motor<T>(j.col(i)));
            absolute_jacobian.setCoefficient(0, i + dof / 2,
                                             second_jacobian.getCoefficient(0, i) * motor + second_motor * Motor<T>(j.col(i + dof / 2)));
        }

        return absolute_jacobian;
    }

    template <class T, int dof>
    PointPair<T> DualManipulator<T, dof>::getEEPointPair(const Eigen::Vector<T, dof / 2> &position_first,
                                                         const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return getFirstEEMotor(position_first).apply(Point<T>()) ^ getSecondEEMotor(position_second).apply(Point<T>());
    }

    template <class T, int dof>
    MultivectorMatrix<T, PointPair, 1, dof> DualManipulator<T, dof>::getPointPairJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                          const Eigen::Vector<T, dof / 2> &position_second) const
    {
        Motor<T> first_motor = getFirstEEMotor(position_first);
        Motor<T> second_motor = getSecondEEMotor(position_second);

        Point<T> first_point = first_motor.apply(Point<T>());
        Point<T> second_point = second_motor.apply(Point<T>());

        MultivectorMatrix<T, Motor, 1, dof / 2> first_jacobian = this->computeKinematicChainAnalyticJacobian(first_ee_joint_name_, position_first);
        MultivectorMatrix<T, Motor, 1, dof / 2> second_jacobian = this->computeKinematicChainAnalyticJacobian(second_ee_joint_name_, position_second);

        MultivectorMatrix<T, PointPair, 1, dof> pointpair_jacobian;

        for (unsigned j = 0; j < dof / 2; ++j)
        {
            pointpair_jacobian.getCoefficient(0, j) =
              ((getFirstBaseMotor() * first_jacobian.getCoefficient(0, j) * E0<T>(TypeTraits<T>::Value(1.0)) * first_motor.reverse()) ^ second_point) +
              ((first_motor * E0<T>(TypeTraits<T>::Value(1.0)) * Motor<T>(getFirstBaseMotor() * first_jacobian.getCoefficient(0, j)).reverse()) ^
               second_point);

            pointpair_jacobian.getCoefficient(0, j + j) =
              (first_point ^ (getSecondBaseMotor() * second_jacobian.getCoefficient(0, j) * E0<T>(TypeTraits<T>::Value(1.0)) * second_motor.reverse())) +
              (first_point ^
               (second_motor * E0<T>(TypeTraits<T>::Value(1.0)) * Motor<T>(getSecondBaseMotor() * second_jacobian.getCoefficient(0, j)).reverse()));
        }

        return pointpair_jacobian;
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getRelativeVelocityManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getRelativeVelocityManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getRelativeVelocityManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                      const Eigen::Vector<T, dof / 2> &position_second) const
    {
        Eigen::Matrix<T, 6, dof> jacobian = getRelativeGeometricJacobian(position_first, position_second).embed();

        return jacobian * jacobian.transpose();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getRelativeForceManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getRelativeForceManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getRelativeForceManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                   const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return getRelativeVelocityManipulability(position_first, position_second).inverse();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getRelativeDynamicManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getRelativeDynamicManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getRelativeDynamicManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                     const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return Eigen::Matrix<T, 6, 6>::Zero();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getAbsoluteVelocityManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getAbsoluteVelocityManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getAbsoluteVelocityManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                      const Eigen::Vector<T, dof / 2> &position_second) const
    {
        Eigen::Matrix<T, 6, dof> jacobian = getAbsoluteGeometricJacobian(position_first, position_second).embed();

        return jacobian * jacobian.transpose();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getAbsoluteForceManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getAbsoluteForceManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getAbsoluteForceManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                   const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return getAbsoluteVelocityManipulability(position_first, position_second).inverse();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getAbsoluteDynamicManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getAbsoluteDynamicManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> DualManipulator<T, dof>::getAbsoluteDynamicManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                     const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return Eigen::Matrix<T, 6, 6>::Zero();
    }

    template <class T, int dof>
    Eigen::Matrix<T, dof, 1> DualManipulator<T, dof>::getJointTorques(const Eigen::Vector<T, dof> &positions, const Eigen::Vector<T, dof> &velocities,
                                                                      const Eigen::Vector<T, dof> &accelerations, const T &gravity)
    {
        Eigen::Matrix<T, dof, 1> torques;

        torques.topRows(dof / 2) = this->template computeInverseDynamics<dof / 2>(positions.topRows(dof / 2),      //
                                                                                  velocities.topRows(dof / 2),     //
                                                                                  accelerations.topRows(dof / 2),  //
                                                                                  gravity,                         //
                                                                                  Wrench<T>::Zero(),               //
                                                                                  first_ee_joint_name_);

        torques.bottomRows(dof / 2) = this->template computeInverseDynamics<dof / 2>(positions.bottomRows(dof / 2),      //
                                                                                     velocities.bottomRows(dof / 2),     //
                                                                                     accelerations.bottomRows(dof / 2),  //
                                                                                     gravity,                            //
                                                                                     Wrench<T>::Zero(),                  //
                                                                                     second_ee_joint_name_);

        return torques;
    }

}  // namespace gafro