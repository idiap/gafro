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

#include <gafro/robot.hpp>
//
#include <orwell/RobotModel.hpp>

namespace gafro_control
{

    template <int dof>
    class RobotModel : public orwell::RobotModel<dof>
    {
      public:
        RobotModel(const std::shared_ptr<gafro::Manipulator<double, dof>> &manipulator);

        virtual ~RobotModel();

        using Vector = typename orwell::RobotModel<dof>::Vector;

        // interface methods

        void computeCartesianState(const orwell::RobotState<dof> &robot_state);

        const Eigen::Matrix<double, 6, dof> &getJacobian() const;

        // gafro specific methods

        const gafro::Motor<double> &getEEMotor() const;

        const gafro::Twist<double> &getEETwist() const;

        std::shared_ptr<gafro::Manipulator<double, dof>> getManipulator();

        Vector computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration, const double &gravity);

      protected:
      private:
        std::shared_ptr<gafro::Manipulator<double, dof>> manipulator_;

        Eigen::Matrix<double, 6, dof> jacobian_;

        gafro::Motor<double> ee_motor_;

        gafro::Twist<double> ee_twist_;

      public:
        using Ptr = std::shared_ptr<RobotModel>;

        template <template <class> class Manipulator>
        static Ptr create();
    };

}  // namespace gafro_control

#include <gafro_control/RobotModel.hxx>