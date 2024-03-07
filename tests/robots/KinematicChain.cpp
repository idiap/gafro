#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Default kinematic chain", "[KinematicChain]" )
{
    KinematicChain<double> chain;

    REQUIRE( chain.getDoF() == 0 );

    const std::map<int, Motor<double>>& motors = chain.getFixedMotors();
    REQUIRE( motors.size() == 0 );

    const std::vector<const Joint<double>*>& joints = chain.getActuatedJoints();
    REQUIRE( joints.size() == 0 );
}


TEST_CASE( "Kinematic chain with joints", "[KinematicChain]" )
{
    Motor<double> translator(Translator<double>::Generator({ 0.0, 1.0, 0.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint1 = std::make_unique<RevoluteJoint<double>>();
    joint1->setName("joint1");
    joint1->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint2 = std::make_unique<RevoluteJoint<double>>();
    joint2->setName("joint2");
    joint2->setFrame(translator);
    joint2->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint3 = std::make_unique<RevoluteJoint<double>>();
    joint3->setName("joint3");
    joint3->setFrame(translator);
    joint3->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));

    KinematicChain<double> chain;

    chain.addActuatedJoint(joint1.get());
    chain.addActuatedJoint(joint2.get());
    chain.addActuatedJoint(joint3.get());

    REQUIRE( chain.getDoF() == 3 );

    const std::map<int, Motor<double>>& motors = chain.getFixedMotors();
    REQUIRE( motors.size() == 0 );

    const std::vector<const Joint<double>*>& joints = chain.getActuatedJoints();
    REQUIRE( joints.size() == 3 );
    REQUIRE( joints[0]->getName() == "joint1" );
    REQUIRE( joints[1]->getName() == "joint2" );
    REQUIRE( joints[2]->getName() == "joint3" );

    SECTION( "Set fixed motors" )
    {
        std::map<int, Motor<double>> fixed_motors;
        fixed_motors[0] = Motor<double>();
        fixed_motors[1] = Motor<double>();
        fixed_motors[2] = Motor<double>();

        chain.setFixedMotors(fixed_motors);

        const std::map<int, Motor<double>>& motors = chain.getFixedMotors();
        REQUIRE( motors.size() == 3 );
    }

    SECTION( "Compute motor" )
    {
        Eigen::Vector<double, 3> position({0.0, 0.0, M_PI / 2.0});
        Motor<double> motor = chain.computeMotor(position);

        REQUIRE( motor.get<blades::scalar>() == Approx(0.7071067812) );
        REQUIRE( motor.get<blades::e23>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e13>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e12>() == Approx(-0.7071067812) );
        REQUIRE( motor.get<blades::e1i>() == Approx(-0.7071067812) );
        REQUIRE( motor.get<blades::e2i>() == Approx(-0.7071067812) );
        REQUIRE( motor.get<blades::e3i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e123i>() == Approx(0.0) );
    }

    SECTION( "Compute motors" )
    {
        for (int i = 0; i < 3; ++i)
        {
            Motor<double> motor = chain.computeMotor(i, M_PI / 2.0);
            Motor<double> ref = joints[i]->getMotor(M_PI / 2.0);

            REQUIRE( motor.get<blades::scalar>() == Approx(ref.get<blades::scalar>()) );
            REQUIRE( motor.get<blades::e23>() == Approx(ref.get<blades::e23>()) );
            REQUIRE( motor.get<blades::e13>() == Approx(ref.get<blades::e13>()) );
            REQUIRE( motor.get<blades::e12>() == Approx(ref.get<blades::e12>()) );
            REQUIRE( motor.get<blades::e1i>() == Approx(ref.get<blades::e1i>()) );
            REQUIRE( motor.get<blades::e2i>() == Approx(ref.get<blades::e2i>()) );
            REQUIRE( motor.get<blades::e3i>() == Approx(ref.get<blades::e3i>()) );
            REQUIRE( motor.get<blades::e123i>() == Approx(ref.get<blades::e123i>()) );
        }
    }

    SECTION( "Compute motor derivatives" )
    {
        for (int i = 0; i < 3; ++i)
        {
            Motor<double> motor = chain.computeMotorDerivative(i, M_PI / 2.0);
            Motor<double> ref = joints[i]->getMotorDerivative(M_PI / 2.0);

            REQUIRE( motor.get<blades::scalar>() == Approx(ref.get<blades::scalar>()) );
            REQUIRE( motor.get<blades::e23>() == Approx(ref.get<blades::e23>()) );
            REQUIRE( motor.get<blades::e13>() == Approx(ref.get<blades::e13>()) );
            REQUIRE( motor.get<blades::e12>() == Approx(ref.get<blades::e12>()) );
            REQUIRE( motor.get<blades::e1i>() == Approx(ref.get<blades::e1i>()) );
            REQUIRE( motor.get<blades::e2i>() == Approx(ref.get<blades::e2i>()) );
            REQUIRE( motor.get<blades::e3i>() == Approx(ref.get<blades::e3i>()) );
            REQUIRE( motor.get<blades::e123i>() == Approx(ref.get<blades::e123i>()) );
        }
    }

    SECTION( "Compute analytic Jacobian" )
    {
        Eigen::Vector<double, 3> position({0.0, 0.0, M_PI / 2.0});

        MultivectorMatrix<double, Motor, 1, 3> jacobian = chain.computeAnalyticJacobian(position);

        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::scalar>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e12>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e1i>() == Approx(0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e2i>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e3i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e123i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::scalar>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e12>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e1i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e3i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e123i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::scalar>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e12>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e1i>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e2i>() == Approx(0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e3i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e123i>() == Approx(0.0) );
    }

    SECTION( "Compute geometric Jacobian" )
    {
        Eigen::Vector<double, 3> position({0.0, 0.0, M_PI / 2.0});

        MultivectorMatrix<double, MotorGenerator, 1, 3> jacobian = chain.computeGeometricJacobian(position);

        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e1i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e3i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e1i>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e3i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e1i>() == Approx(2.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e3i>() == Approx(0.0) );
    }

    SECTION( "Compute geometric Jacobian body" )
    {
        Motor<double> translator(Translator<double>::Generator({ 0.0, 1.0, 0.0 }));
        chain.addFixedMotor(translator);

        Eigen::Vector<double, 3> position({0.0, 0.0, M_PI / 2.0});

        MultivectorMatrix<double, MotorGenerator, 1, 3> jacobian = chain.computeGeometricJacobianBody(position);

        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e1i>() == Approx(-1.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e2i>() == Approx(2.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e3i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e1i>() == Approx(-1.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e2i>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e3i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e1i>() == Approx(-1.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e3i>() == Approx(0.0) );
    }
}


TEST_CASE( "Kinematic chain with fixed motors", "[KinematicChain]" )
{
    Motor<double> translator(Translator<double>::Generator({ 0.0, 1.0, 0.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint1 = std::make_unique<RevoluteJoint<double>>();
    joint1->setName("joint1");
    joint1->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint2 = std::make_unique<RevoluteJoint<double>>();
    joint2->setName("joint2");
    joint2->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint3 = std::make_unique<RevoluteJoint<double>>();
    joint3->setName("joint3");
    joint3->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));

    KinematicChain<double> chain;

    chain.addActuatedJoint(joint1.get());
    chain.addFixedMotor(translator);
    chain.addActuatedJoint(joint2.get());
    chain.addFixedMotor(translator);
    chain.addActuatedJoint(joint3.get());

    REQUIRE( chain.getDoF() == 3 );

    const std::map<int, Motor<double>>& motors = chain.getFixedMotors();
    REQUIRE( motors.size() == 2 );

    const std::vector<const Joint<double>*>& joints = chain.getActuatedJoints();
    REQUIRE( joints.size() == 3 );
    REQUIRE( joints[0]->getName() == "joint1" );
    REQUIRE( joints[1]->getName() == "joint2" );
    REQUIRE( joints[2]->getName() == "joint3" );

    SECTION( "Compute motor" )
    {
        Eigen::Vector<double, 3> position({0.0, 0.0, M_PI / 2.0});
        Motor<double> motor = chain.computeMotor(position);

        REQUIRE( motor.get<blades::scalar>() == Approx(0.7071067812) );
        REQUIRE( motor.get<blades::e23>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e13>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e12>() == Approx(-0.7071067812) );
        REQUIRE( motor.get<blades::e1i>() == Approx(-0.7071067812) );
        REQUIRE( motor.get<blades::e2i>() == Approx(-0.7071067812) );
        REQUIRE( motor.get<blades::e3i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e123i>() == Approx(0.0) );
    }

    SECTION( "Compute motors" )
    {
        Motor<double> motor = chain.computeMotor(0, M_PI / 2.0);

        REQUIRE( motor.get<blades::scalar>() == Approx(0.7071067812) );
        REQUIRE( motor.get<blades::e23>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e13>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e12>() == Approx(-0.7071067812) );
        REQUIRE( motor.get<blades::e1i>() == Approx(0.353553) );
        REQUIRE( motor.get<blades::e2i>() == Approx(-0.353553) );
        REQUIRE( motor.get<blades::e3i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e123i>() == Approx(0.0) );

        motor = chain.computeMotor(1, M_PI / 2.0);

        REQUIRE( motor.get<blades::scalar>() == Approx(0.7071067812) );
        REQUIRE( motor.get<blades::e23>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e13>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e12>() == Approx(-0.7071067812) );
        REQUIRE( motor.get<blades::e1i>() == Approx(0.353553) );
        REQUIRE( motor.get<blades::e2i>() == Approx(-0.353553) );
        REQUIRE( motor.get<blades::e3i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e123i>() == Approx(0.0) );

        motor = chain.computeMotor(2, M_PI / 2.0);

        REQUIRE( motor.get<blades::scalar>() == Approx(0.7071067812) );
        REQUIRE( motor.get<blades::e23>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e13>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e12>() == Approx(-0.7071067812) );
        REQUIRE( motor.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e3i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e123i>() == Approx(0.0) );
    }

    SECTION( "Compute motor derivatives" )
    {
        Motor<double> motor = chain.computeMotorDerivative(0, M_PI / 2.0);

        REQUIRE( motor.get<blades::scalar>() == Approx(-0.3535533906) );
        REQUIRE( motor.get<blades::e23>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e13>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e12>() == Approx(-0.3535533906) );
        REQUIRE( motor.get<blades::e1i>() == Approx(0.1767766953) );
        REQUIRE( motor.get<blades::e2i>() == Approx(0.1767766953) );
        REQUIRE( motor.get<blades::e3i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e123i>() == Approx(0.0) );

        motor = chain.computeMotorDerivative(1, M_PI / 2.0);

        REQUIRE( motor.get<blades::scalar>() == Approx(-0.3535533906) );
        REQUIRE( motor.get<blades::e23>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e13>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e12>() == Approx(-0.3535533906) );
        REQUIRE( motor.get<blades::e1i>() == Approx(0.1767766953) );
        REQUIRE( motor.get<blades::e2i>() == Approx(0.1767766953) );
        REQUIRE( motor.get<blades::e3i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e123i>() == Approx(0.0) );

        motor = chain.computeMotorDerivative(2, M_PI / 2.0);

        REQUIRE( motor.get<blades::scalar>() == Approx(-0.3535533906) );
        REQUIRE( motor.get<blades::e23>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e13>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e12>() == Approx(-0.3535533906) );
        REQUIRE( motor.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e3i>() == Approx(0.0) );
        REQUIRE( motor.get<blades::e123i>() == Approx(0.0) );
    }

    SECTION( "Compute analytic Jacobian" )
    {
        Eigen::Vector<double, 3> position({0.0, 0.0, M_PI / 2.0});

        MultivectorMatrix<double, Motor, 1, 3> jacobian = chain.computeAnalyticJacobian(position);

        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::scalar>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e12>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e1i>() == Approx(0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e2i>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e3i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e123i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::scalar>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e12>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e1i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e3i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e123i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::scalar>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e12>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e1i>() == Approx(-0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e2i>() == Approx(0.353553) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e3i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e123i>() == Approx(0.0) );
    }

    SECTION( "Compute geometric Jacobian" )
    {
        Eigen::Vector<double, 3> position({0.0, 0.0, M_PI / 2.0});

        MultivectorMatrix<double, MotorGenerator, 1, 3> jacobian = chain.computeGeometricJacobian(position);

        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e1i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e3i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e1i>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e3i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e1i>() == Approx(2.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e3i>() == Approx(0.0) );
    }

    SECTION( "Compute geometric Jacobian body" )
    {
        Motor<double> translator(Translator<double>::Generator({ 0.0, 1.0, 0.0 }));
        chain.addFixedMotor(translator);

        Eigen::Vector<double, 3> position({0.0, 0.0, M_PI / 2.0});

        MultivectorMatrix<double, MotorGenerator, 1, 3> jacobian = chain.computeGeometricJacobianBody(position);

        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e1i>() == Approx(-1.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 0).get<blades::e3i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e1i>() == Approx(-1.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 1).get<blades::e3i>() == Approx(0.0) );

        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e23>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e13>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e12>() == Approx(1.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e1i>() == Approx(-1.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e2i>() == Approx(0.0) );
        REQUIRE( jacobian.getCoefficient(0, 2).get<blades::e3i>() == Approx(0.0) );
    }
}
