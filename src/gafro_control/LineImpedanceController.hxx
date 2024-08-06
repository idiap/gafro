#pragma once

#include <gafro_control/LineImpedanceController.hpp>
#include <gafro_control/RobotModel.hpp>

namespace gafro_control
{

    template <int dof>
    LineImpedanceController<dof>::LineImpedanceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : orwell::CartesianImpedanceController<dof, gafro::Line<double>>(interface, name)
    {}

    template <int dof>
    LineImpedanceController<dof>::~LineImpedanceController() = default;

    template <int dof>
    void LineImpedanceController<dof>::computeStateError()
    {
        auto robot_model = std::dynamic_pointer_cast<gafro_control::RobotModel<dof>>(this->getRobotModel());

        gafro::Motor<double> ee_motor = robot_model->getEEMotor();

        gafro::Line<double> target_line = gafro::Motor<double>(ee_motor.reverse()).apply(this->getReference());

        gafro::Motor<double> motor = gafro::Line<double>::Z().getMotor(target_line);

        Eigen::Vector<double, 6> position_error = motor.log().evaluate().vector();
        Eigen::Vector<double, 6> velocity_error = -robot_model->getEETwist().vector();

        this->setPositionError(position_error);
        this->setVelocityError(velocity_error);
        this->setAccelerationError(gafro::Line<double>::Z().vector());
    }

}  // namespace gafro_control