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