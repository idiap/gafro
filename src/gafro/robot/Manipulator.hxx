/*
    Copyright (c) 2022 Idiap Research Institute, http://www.idiap.ch/
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of gafro.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <gafro/algebra/MultivectorMatrix.hpp>
//
#include <gafro/robot/EndEffector.hpp>
#include <gafro/robot/Manipulator.hpp>

namespace gafro
{

    template <class T, int dof>
    Manipulator<T, dof>::Manipulator(const std::array<std::array<T, 3>, dof> &dh_parameters, const std::array<T, 3> &end_effector_link,
                                     std::unique_ptr<EndEffector<T>> &&end_effector)
      : end_effector_(std::move(end_effector))
    {
        for (int i = 0; i < dof; ++i)
        {
            joints_[i] = Joint<T>(dh_parameters[i]);
        }

        end_effector_link_ =
          Motor<T>(Translator<T>(typename Translator<T>::Generator({ end_effector_link[0], 0.0, end_effector_link[1] })),
                   Rotor<T>(typename Rotor<T>::Generator(Eigen::Matrix<T, 3, 1>({ T(0.0), T(0.0), T(1.0) })), end_effector_link[2]));
    }

    template <class T, int dof>
    Manipulator<T, dof>::Manipulator(const std::array<std::array<T, 6>, dof> &parameters, const std::array<int, dof> &axis)
    {
        for (int i = 0; i < dof; ++i)
        {
            joints_[i] = Joint<T>(parameters[i], axis[i]);
        }
    }

    template <class T, int dof>
    Manipulator<T, dof>::Manipulator(const Manipulator &other)
    {
        joints_ = other.joints_;
        end_effector_link_ = other.end_effector_link_;
        base_ = other.base_;
    }
    template <class T, int dof>
    Manipulator<T, dof>::~Manipulator()
    {}

    template <class T, int dof>
    typename Manipulator<T, dof>::State Manipulator<T, dof>::getRandomConfiguration() const
    {
        return joint_limits_min_ + (State::Random().array().abs() * (joint_limits_max_ - joint_limits_min_).array()).matrix();
    }

    template <class T, int dof>
    void Manipulator<T, dof>::setJointLimits(const State &joint_limits_min, const State &joint_limits_max)
    {
        joint_limits_min_ = joint_limits_min;
        joint_limits_max_ = joint_limits_max;
    }

    template <class T, int dof>
    const typename Manipulator<T, dof>::State &Manipulator<T, dof>::getJointLimitsMin() const
    {
        return joint_limits_min_;
    }

    template <class T, int dof>
    const typename Manipulator<T, dof>::State &Manipulator<T, dof>::getJointLimitsMax() const
    {
        return joint_limits_max_;
    }

    template <class T, int dof>
    void Manipulator<T, dof>::setBase(const Motor<T> &base)
    {
        base_ = base;
    }

    template <class T, int dof>
    const Motor<T> &Manipulator<T, dof>::getBase() const
    {
        return base_;
    }

    template <class T, int dof>
    Link<T> &Manipulator<T, dof>::getLink(const unsigned &index)
    {
        if (index >= dof)
        {
            throw std::runtime_error("Manpulator: index larger than dof");
        }

        return links_[index];
    }

    template <class T, int dof>
    const Link<T> &Manipulator<T, dof>::getLink(const unsigned &index) const
    {
        if (index >= dof)
        {
            throw std::runtime_error("Manpulator: index larger than dof");
        }

        return links_[index];
    }

    template <typename T, int dof>
    const std::array<Joint<T>, dof> &Manipulator<T, dof>::getJoints() const
    {
        return joints_;
    }

    template <typename T, int dof>
    const std::array<Link<T>, dof> &Manipulator<T, dof>::getLinks() const
    {
        return links_;
    }

    template <typename T, int dof>
    const Motor<T> &Manipulator<T, dof>::getEndEffectorLink() const
    {
        return end_effector_link_;
    }

    template <class T, int dof>
    Motor<T> Manipulator<T, dof>::getJointMotor(const unsigned &id, const State &position) const
    {
        Motor<T> motor = base_;

        for (int i = 0; i <= id && i < dof; ++i)
        {
            motor *= joints_[i].getMotor(position.coeff(i, 0));
        }

        return motor;
    }

    template <class T, int dof>
    const std::array<Motor<T>, dof> &Manipulator<T, dof>::getJointMotors(const State &position) const
    {
        motors_[0] = base_ * joints_[0].getMotor(position[0]);

        for (int i = 1; i < dof; ++i)
        {
            motors_[i] = motors_[i - 1] * joints_[i].getMotor(position[i]);
        }

        return motors_;
    }

    template <class T, int dof>
    Motor<T> Manipulator<T, dof>::getEEMotor(const State &position) const
    {
        // return Motor<T>(getJointMotors(position).back()) * getEndEffectorLink();
        return end_effector_->addTransform(Motor<T>(getJointMotors(position).back()) * getEndEffectorLink());
    }

    template <class T, int dof>
    Eigen::Matrix<T, 8, dof> Manipulator<T, dof>::getJointJacobian(const unsigned &id, const State &position, const Motor<T> &base) const
    {
        std::array<Motor<T>, dof> m;

        Eigen::Matrix<T, 8, dof> jacobian = Eigen::Matrix<T, 8, dof>::Zero();

        std::array<Motor<T>, dof> motors = getJointMotors(position);

        Motor<T> motor;

        for (unsigned i = 1; i < dof; ++i)
        {
            auto p = motors[i - 1] * joints_[i].getFrame() * getJoint(i).getRotationRotorGenerator() * Scalar<T>(-0.5) *
                     joints_[i].getRotor(position.coeff(dof - 1, 0));
        }

        for (unsigned i = 0; i < id + 1; ++i)
        {
            jacobian.col(i - 1) = motor.vector();
        }

        return jacobian;
    }

    // template <class T, int dof>
    // Motor<T> Manipulator<T, dof>::getJointMotor(const unsigned &id) const
    // {
    //     Motor<T> motor = base_;

    //     for (int i = 0; i <= id && i < dof; ++i)
    //     {
    //         motor = motor * joints_[i].getMotor(this->getState()[i]);
    //     }

    //     return motor;
    // }

    template <class T, int dof>
    MultivectorMatrix<Motor<T>, 1, dof> Manipulator<T, dof>::getEEAnalyticJacobian(const State &position) const
    {
        MultivectorMatrix<Motor<T>, 1, dof> jacobian = MultivectorMatrix<Motor<T>, 1, dof>::Zero();

        jacobian.fill(base_);

        for (unsigned i = 0; i < dof; ++i)
        {
            auto motor = joints_[i].getMotor(position[i]);

            jacobian[i] *= getJoint(i).getFrame() * Scalar<T>(T(-0.5)) * getJoint(i).getRotationRotorGenerator() * getJoint(i).getRotor(position[i]);

            for (unsigned j = 0; j < dof; ++j)
            {
                if (j != i)
                {
                    jacobian[j] *= motor;
                }
            }
        }

        for (unsigned i = 0; i < dof; ++i)
        {
            // jacobian[i] = jacobian[i] * getEndEffectorLink();
            jacobian[i] = end_effector_->addTransform(jacobian[i] * getEndEffectorLink());
        }

        return jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<typename Motor<T>::Generator, 1, dof> Manipulator<T, dof>::getEEGeometricJacobian(const State &position) const
    {
        MultivectorMatrix<typename Motor<T>::Generator, 1, dof> jacobian;

        Motor<T> joint_motor = base_;

        for (unsigned j = 0; j < dof; ++j)
        {
            Motor<T> motor = joint_motor * joints_[j].getFrame();

            jacobian.coeffRef(0, j) = motor * joints_[j].getAxis() * motor.reverse();

            joint_motor *= joints_[j].getMotor(position.coeff(j, 0));
        }

        return jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<Motor<T>, dof, dof> Manipulator<T, dof>::getFullGeometricJacobian(const State &position) const
    {
        MultivectorMatrix<Motor<T>, dof, dof> jacobian = MultivectorMatrix<Motor<T>, dof, dof>::Zero();

        Motor<T> joint_motor = base_;

        for (unsigned j = 0; j < dof; ++j)
        {
            Motor<T> motor = joint_motor * joints_[j].getFrame();

            jacobian.block(j, j, dof - j, 1).fill(motor * joints_[j].getAxis() * motor.reverse());

            joint_motor *= joints_[j].getMotor(position.coeff(j, 0));
        }

        return jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<Motor<T>, 1, dof> Manipulator<T, dof>::getEEGeometricJacobianTimeDerivative(const State &position, const State &velocity) const
    {
        std::vector<typename Motor<T>::Generator> axes;
        axes.reserve(dof);

        std::array<Motor<T>, dof> joint_motors = getJointMotors(position);

        for (unsigned j = 0; j < dof; ++j)
        {
            Motor<T> motor;

            if (j > 0)
            {
                motor = joint_motors[j - 1] * joints_[j].getFrame();
            }

            axes[j] = motor * joints_[j].getAxis() * motor.reverse();
        }

        using Commutator = typename GeometricProduct<typename Motor<T>::Generator, typename Motor<T>::Generator>::Type;

        MultivectorMatrix<Commutator, dof, dof> jacobian_commutator = MultivectorMatrix<Commutator, dof, dof>::Zero();

        for (unsigned r = 0; r < dof; ++r)
        {
            for (unsigned c = 0; c <= r; ++c)
            {
                jacobian_commutator.coeffRef(r, c) = axes[r] * axes[c] - axes[c] * axes[r];
            }
        }

        MultivectorMatrix<typename Scalar<T>::Base, dof, 1> v = (0.5 * velocity).template cast<Scalar<T>>().template cast<typename Scalar<T>::Base>();

        MultivectorMatrix<typename Motor<T>::Base, dof, 1> jacobian_time_derivative = jacobian_commutator * v;

        return jacobian_time_derivative.template cast<Motor<T>>().transpose();
    }

    template <class T, int dof>
    MultivectorMatrix<Motor<T>, dof, dof> Manipulator<T, dof>::getFullGeometricJacobianTimeDerivative(const State &position,
                                                                                                      const State &velocity) const
    {
        MultivectorMatrix<Motor<T>, dof, dof> full_jacobian_time_derivative = MultivectorMatrix<Motor<T>, dof, dof>::Zero();

        MultivectorMatrix<Motor<T>, 1, dof> jacobian_time_derivative = getEEGeometricJacobianTimeDerivative(position, velocity);

        for (unsigned j = 0; j < dof; ++j)
        {
            full_jacobian_time_derivative.block(j, j, dof - j, 1).fill(jacobian_time_derivative.coeff(0, j));
        }

        return full_jacobian_time_derivative;
    }

    template <class T, int dof>
    MultivectorMatrix<typename InnerProduct<Point<T>, typename Motor<T>::Generator>::Type, dof, dof> Manipulator<T, dof>::getProjectedMassMatrix(
      const State &position) const
    {
        MultivectorMatrix<typename InnerProduct<Point<T>, typename Motor<T>::Generator>::Type, dof, dof> projected_mass_matrix =
          MultivectorMatrix<typename InnerProduct<Point<T>, typename Motor<T>::Generator>::Type, dof, dof>::Zero();

        std::vector<typename Motor<T>::Generator> axes;
        axes.reserve(dof);

        std::array<Motor<T>, dof> joint_motors = getJointMotors(position);

        for (unsigned j = 0; j < dof; ++j)
        {
            Motor<T> motor;

            if (j > 0)
            {
                motor = joint_motors[j - 1];
            }

            axes[j] = Motor<T>(motor * joints_[j].getFrame()) * joints_[j].getAxis() * Motor<T>(motor * joints_[j].getFrame()).reverse();

            Point<T> com = Motor<T>(motor * joints_[j].getMotor(position[j]) * links_[j].getCenterOfMass()).apply(Point<T>());

            for (unsigned k = 0; k <= j; ++k)
            {
                projected_mass_matrix.coeffRef(j, k) = (com | axes[k]);
            }
        }

        return projected_mass_matrix;
    }

    template <class T, int dof>
    MultivectorMatrix<Point<T>, dof, dof> Manipulator<T, dof>::getLinkCoMs(const State &position) const
    {
        MultivectorMatrix<Point<T>, dof, dof> link_coms = MultivectorMatrix<Point<T>, dof, dof>::Zero();

        std::array<Motor<T>, dof> joint_motors = getJointMotors(position);

        for (unsigned j = 0; j < dof; ++j)
        {
            Motor<T> motor(joint_motors[j] * links_[j].getCenterOfMass());

            link_coms.coeffRef(j, j) = motor * Point<T>() * motor.reverse();
        }

        return link_coms;
    }

    template <class T, int dof>
    MultivectorMatrix<Scalar<T>, dof, dof> Manipulator<T, dof>::getMassMatrix(const State &position) const
    {
        MultivectorMatrix<typename InnerProduct<Point<T>, typename Motor<T>::Generator>::Type, dof, dof> projected_mass_matrix =
          getProjectedMassMatrix(position);

        MultivectorMatrix<typename Scalar<T>::Base, dof, dof> link_masses = MultivectorMatrix<typename Scalar<T>::Base, dof, dof>::Zero();

        for (unsigned i = 0; i < dof; ++i)
        {
            link_masses.coeffRef(i, i) = Scalar<T>(links_[i].getMass());
        }

        MultivectorMatrix<typename Scalar<T>::Base, dof, dof> mass_matrix = MultivectorMatrix<typename Scalar<T>::Base, dof, dof>::Zero();

        mass_matrix = (projected_mass_matrix.transpose() * link_masses * projected_mass_matrix);

        return mass_matrix.template cast<Scalar<T>>();
    }

    template <class T, int dof>
    MultivectorMatrix<Scalar<T>, dof, dof> Manipulator<T, dof>::getInertiaMatrix(const State &position) const
    {
        MultivectorMatrix<Scalar<T>, dof, dof> inertia_matrix = MultivectorMatrix<Scalar<T>, dof, dof>::Zero();

        MultivectorMatrix<Motor<T>, dof, dof> jacobian = getFullGeometricJacobian(position);

        std::array<Motor<T>, dof> joint_motors = getJointMotors(position);

        for (unsigned i = 0; i < dof; ++i)
        {
            Rotor<T> rotor = joint_motors[i].getRotor();

            MultivectorMatrix<typename Rotor<T>::Generator::Base, 1, dof> bjac;
            MultivectorMatrix<typename Rotor<T>::Generator::Base, 1, dof> bjac_inertia;

            for (unsigned j = 0; j < dof; ++j)
            {
                typename Rotor<T>::Generator b = jacobian.coeff(i, j).getRotor().log();

                b = Rotor<T>(rotor.reverse()).apply(b).evaluate();

                bjac.coeffRef(0, j) = b;
                bjac_inertia.coeffRef(0, j) = links_[i].getInertia()(b);
            }

            inertia_matrix += MultivectorMatrix<typename Rotor<T>::Base, dof, dof>(bjac.transpose() * bjac_inertia).template extract<Scalar<T>>();
        }

        return inertia_matrix;
    }

    template <class T, int dof>
    MultivectorMatrix<Scalar<T>, dof, 1> Manipulator<T, dof>::getInertiaMatrixTimeDerivative(const State &position, const State &velocity) const
    {
        MultivectorMatrix<Scalar<T>, dof, 1> inertia_matrix_time_derivative = MultivectorMatrix<Scalar<T>, dof, 1>::Zero();

        MultivectorMatrix<Motor<T>, dof, dof> jacobian = getFullGeometricJacobian(position);
        MultivectorMatrix<Motor<T>, dof, dof> jacobian_time_derivative = getFullGeometricJacobianTimeDerivative(position, velocity);

        std::array<Motor<T>, 7> joint_motors = getJointMotors(position);

        for (unsigned i = 0; i < dof; ++i)
        {
            Rotor<T> rotor = joint_motors[i].getRotor();

            MultivectorMatrix<typename Rotor<T>::Generator::Base, 1, dof> bjac;
            MultivectorMatrix<typename Rotor<T>::Generator::Base, 1, dof> bjac_inertia_dt;

            for (unsigned j = 0; j < dof; ++j)
            {
                bjac.coeffRef(0, j) = Motor<T>(Rotor<T>(rotor.reverse()).apply(jacobian.coeff(i, j))).getRotor().log();

                typename Rotor<T>::Generator b_dt = jacobian_time_derivative.coeff(i, j).getRotor().log();

                b_dt = Rotor<T>(rotor.reverse()).apply(b_dt).evaluate();

                bjac_inertia_dt.coeffRef(0, j) = links_[i].getInertia()(b_dt);
            }

            MultivectorMatrix<typename Scalar<T>::Base, dof, 1> v = velocity.template cast<Scalar<T>>().template cast<typename Scalar<T>::Base>();
            MultivectorMatrix<typename Motor<T>::Base, 1, dof> j = jacobian.row(i).template cast<typename Motor<T>::Base>();

            typename Motor<T>::Base r = (j * v).value();

            const T &w0 = r.template get<blades::e23>();
            const T &w1 = r.template get<blades::e13>();
            const T &w2 = r.template get<blades::e12>();

            Multivector<T, blades::e1, blades::e2, blades::e3> ww0({ w0, -w1, w2 });

            Rotor<T> reversed_rotor = Rotor<T>(rotor.reverse());

            Multivector<T, blades::e1, blades::e2, blades::e3> w = reversed_rotor * ww0 * rotor;
            w = links_[i].getInertia()(w);
            w = rotor * w * reversed_rotor;

            typename Rotor<T>::Generator b = ww0 ^ w;
            Rotor<T> rr = (reversed_rotor * b * rotor) + (bjac_inertia_dt * v).value();

            inertia_matrix_time_derivative += bjac.transpose() * rr;
        }

        return inertia_matrix_time_derivative;
    }

    template <class T, int dof>
    MultivectorMatrix<Scalar<T>, dof, 1> Manipulator<T, dof>::getGravityVector(const State &position) const
    {
        MultivectorMatrix<typename InnerProduct<Point<T>, typename Motor<T>::Generator>::Type, dof, dof> projected_mass_matrix =
          getProjectedMassMatrix(position);

        MultivectorMatrix<typename Scalar<T>::Base, dof, dof> link_masses = MultivectorMatrix<typename Scalar<T>::Base, dof, dof>::Zero();

        for (unsigned i = 0; i < dof; ++i)
        {
            link_masses.coeffRef(i, i) = Scalar<T>(links_[i].getMass());
        }

        MultivectorMatrix<typename E3<T>::Base, dof, 1> gravity = MultivectorMatrix<typename E3<T>::Base, dof, 1>::Zero();
        gravity.fill(E3<T>(T(9.81)));

        MultivectorMatrix<typename Scalar<T>::Base, dof, 1> gravity_vector = MultivectorMatrix<typename Scalar<T>::Base, dof, 1>::Zero();

        gravity_vector = projected_mass_matrix.transpose() * link_masses * gravity;

        std::vector<typename Motor<T>::Generator> axes;
        axes.reserve(dof);

        std::array<Motor<T>, dof> joint_motors = getJointMotors(position);

        for (unsigned j = 0; j < dof; ++j)
        {
            Motor<T> motor;

            if (j > 0)
            {
                motor = joint_motors[j - 1];
            }

            axes[j] = Motor<T>(motor * joints_[j].getFrame()) * joints_[j].getAxis() * Motor<T>(motor * joints_[j].getFrame()).reverse();
        }

        if (end_effector_)
        {
            end_effector_->addGravity(gravity_vector, getEEMotor(position), axes);
        }

        return gravity_vector.template cast<Scalar<T>>();
    }

    template <class T, int dof>
    MultivectorMatrix<Scalar<T>, dof, 1> Manipulator<T, dof>::getCoriolisVector(const State &position, const State &velocity) const
    {
        MultivectorMatrix<Point<T>, dof, dof> link_coms = getLinkCoMs(position);
        MultivectorMatrix<Motor<T>, dof, dof> jacobian = getFullGeometricJacobian(position);
        MultivectorMatrix<Motor<T>, dof, dof> jacobian_time_derivative = getFullGeometricJacobianTimeDerivative(position, velocity);
        MultivectorMatrix<typename InnerProduct<Point<T>, typename Motor<T>::Generator>::Type, dof, dof> projected_mass_matrix =
          getProjectedMassMatrix(position);

        MultivectorMatrix<typename Scalar<T>::Base, dof, dof> link_masses = MultivectorMatrix<typename Scalar<T>::Base, dof, dof>::Zero();

        for (unsigned i = 0; i < dof; ++i)
        {
            link_masses.coeffRef(i, i) = Scalar<T>(links_[i].getMass());
        }

        MultivectorMatrix<typename Scalar<T>::Base, dof, 1> v = velocity.template cast<Scalar<T>>().template cast<typename Scalar<T>::Base>();

        MultivectorMatrix<typename InnerProduct<Point<T>, typename Motor<T>::Generator>::Type, dof, dof> p1 =
          (projected_mass_matrix * v).asDiagonal();

        using Vdot = typename GeometricProduct<typename InnerProduct<Point<T>, typename Motor<T>::Generator>::Type, Motor<T>>::Type;

        MultivectorMatrix<Vdot, dof, dof> vdot = p1 * jacobian.template cast<typename Motor<T>::Base>();

        vdot += link_coms.template cast<typename Point<T>::Base>() * jacobian_time_derivative.template cast<typename Motor<T>::Base>();

        MultivectorMatrix<Vdot, dof, 1> m = vdot * v;

        MultivectorMatrix<typename Sum<Vdot, typename E3<T>::Base>::Type, dof, 1> m2;

        m2.assign(m.unaryExpr([](const Vdot &v) { return (v + E3<T>(9.81)).evaluate(); }));

        MultivectorMatrix<typename InnerProduct<Point<T>, typename Motor<T>::Generator>::Type, dof, dof> p2 =
          projected_mass_matrix.transpose() * link_masses;

        MultivectorMatrix<Scalar<T>, dof, 1> coriolis;
        coriolis.assign(p2 * m);

        return coriolis;
    }

    template <class T, int dof>
    typename Manipulator<T, dof>::State Manipulator<T, dof>::getJointTorques(const State &position, const State &velocity,
                                                                             const State &acceleration) const
    {
        return (getInertiaMatrix(position).embed() + getMassMatrix(position).embed()) * acceleration + getCoriolisVector(position, velocity).embed() +
               getInertiaMatrixTimeDerivative(position, velocity).embed();
    }

    // template <class T, int dof>
    // typename Manipulator<T, dof>::State Manipulator<T, dof>::getJointAccelerations(const State &position, const State &velocity,
    //                                                                                const State &torques) const
    // {
    //     MultivectorMatrix<T, dof, 1> acceleration =
    //       MultivectorMatrix<T, dof, dof>(getInertiaMatrix(position) + getMassMatrix(position)).scalar().inverse().template cast<Multivector<T>>() *
    //       (torques.template cast<Multivector<T>>() - getCoriolisVector(position, velocity) - getInertiaMatrixTimeDerivative(position, velocity));

    //     return acceleration.scalar();
    // }

    // template <class T, int dof>
    // Eigen::Matrix<T, dof, dof> Manipulator<T, dof>::getGeneralizedMassMatrix(const State &position, const State &velocity) const
    // {
    //     return MultivectorMatrix<T, dof, dof>(getInertiaMatrix(position) + getMassMatrix(position)).scalar();
    // }

    // template <class T, int dof>
    // Eigen::Matrix<T, dof, dof> Manipulator<T, dof>::getNonlinearities(const State &position, const State &velocity) const
    // {}

    // template <class T, int dof>
    // T Manipulator<T, dof>::getPotentialEnergy(const State &position) const
    // {
    //     T potential_energy = 0.0;

    //     MultivectorMatrix<T, dof, 1> link_coms = getLinkCoMs(position).diagonal();

    //     for (unsigned j = 0; j < dof; ++j)
    //     {
    //         potential_energy += (link_coms.coeff(j, 0).mv() | (links_[j].getMass() * blades::E3<T>(9.81))).scalar();
    //     }

    //     return potential_energy;
    // }

    // template <class T, int dof>
    // T Manipulator<T, dof>::getKineticEnergy(const State &position, const State &velocity) const
    // {
    //     return T(0.5) * (velocity.template cast<Multivector<T>>().transpose() * (getMassMatrix(position) + getInertiaMatrix(position)) *
    //                      velocity.template cast<Multivector<T>>())
    //
    //                       .scalar();
    // }

    // template <class T, int dof>
    // T Manipulator<T, dof>::getEnergy(const State &position, const State &velocity) const
    // {
    //     return getPotentialEnergy(position) + getKineticEnergy(position, velocity);
    // }

    template <class T, int dof>
    Joint<T> &Manipulator<T, dof>::getJoint(const unsigned &index)
    {
        if (index >= dof)
        {
            throw std::runtime_error("Manpulator: index larger than dof");
        }

        return joints_[index];
    }

    template <class T, int dof>
    const Joint<T> &Manipulator<T, dof>::getJoint(const unsigned &index) const
    {
        if (index >= dof)
        {
            throw std::runtime_error("Manpulator: index larger than dof");
        }

        return joints_[index];
    }

    template <class T, int dof>
    const EndEffector<T> *Manipulator<T, dof>::getEndEffector() const
    {
        return end_effector_.get();
    }

}  // namespace gafro