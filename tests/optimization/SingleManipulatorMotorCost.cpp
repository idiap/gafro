#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE("SingleManipulatorMotorCost", "[SingleManipulatorMotorCost]")
{
    // Create some links
    Translator<double> com(Translator<double>::Generator({ 0.0, 0.0, 0.0 }));

    std::unique_ptr<Link<double>> link1 = std::make_unique<Link<double>>();
    link1->setName("link1");
    link1->setMass(0.1);
    link1->setCenterOfMass(com);
    link1->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    link1->setAxis(Motor<double>::Generator({ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 }));

    std::unique_ptr<Link<double>> link2 = std::make_unique<Link<double>>();
    link2->setName("link2");
    link2->setMass(0.1);
    link2->setCenterOfMass(com);
    link2->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    link2->setAxis(Motor<double>::Generator({ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 }));

    std::unique_ptr<Link<double>> link3 = std::make_unique<Link<double>>();
    link3->setName("link3");
    link3->setMass(0.1);
    link3->setCenterOfMass(com);
    link3->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    link3->setAxis(Motor<double>::Generator({ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 }));

    std::unique_ptr<Link<double>> link4 = std::make_unique<Link<double>>();
    link4->setName("link4");
    link4->setMass(0.1);
    link4->setCenterOfMass(com);
    link4->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    link4->setAxis(Motor<double>::Generator({ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 }));

    // Create some joints
    Translator<double> t(Translator<double>::Generator({ 0.0, 1.0, 0.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint1 = std::make_unique<RevoluteJoint<double>>();
    joint1->setName("joint1");
    joint1->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint1->setLimits(Joint<double>::Limits({ -0.5, 0.5, 1.0, 1.0 }));

    joint1->setFrame(Motor<double>(t));

    std::unique_ptr<RevoluteJoint<double>> joint2 = std::make_unique<RevoluteJoint<double>>();
    joint2->setName("joint2");
    joint2->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint2->setLimits(Joint<double>::Limits({ -0.8, 0.8, 1.0, 1.0 }));
    joint2->setFrame(Motor<double>(t));

    std::unique_ptr<RevoluteJoint<double>> joint3 = std::make_unique<RevoluteJoint<double>>();
    joint3->setName("joint3");
    joint3->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint3->setLimits(Joint<double>::Limits({ -0.8, 0.8, 1.0, 1.0 }));
    joint3->setFrame(Motor<double>(t));

    // Create the system
    System<double> system;

    joint1->setParentLink(link1.get());
    link1->addChildJoint(joint1.get());

    joint1->setChildLink(link2.get());
    link2->setParentJoint(joint1.get());

    joint2->setParentLink(link2.get());
    link2->addChildJoint(joint2.get());

    joint2->setChildLink(link3.get());
    link3->setParentJoint(joint2.get());

    joint3->setParentLink(link3.get());
    link3->addChildJoint(joint3.get());

    joint3->setChildLink(link4.get());
    link4->setParentJoint(joint3.get());

    system.addLink(std::move(link1));
    system.addLink(std::move(link2));
    system.addLink(std::move(link3));
    system.addLink(std::move(link4));

    system.addJoint(std::move(joint1));
    system.addJoint(std::move(joint2));
    system.addJoint(std::move(joint3));

    system.finalize();

    // Create the manipulator
    Manipulator<double, 3> manipulator(std::move(system), "joint3");

    // Create the cost function
    Eigen::Vector<double, 3> target_position({ 0.0, M_PI / 2.0, 0.0 });

    Motor<double> ee_target_motor = manipulator.getEEMotor(target_position);

    SingleManipulatorMotorCost<double, 3> cost_function(&manipulator, ee_target_motor);

    SECTION("Get error")
    {
        auto error = cost_function.getError({0.0, 0.0, 0.0});

        REQUIRE(error.rows() == 6);
        REQUIRE(error(0, 0) == Approx(0.0));
        REQUIRE(error(1, 0) == Approx(0.0));
        REQUIRE(error(2, 0) == Approx(-1.5708));
        REQUIRE(error(3, 0) == Approx(1.0));
        REQUIRE(error(4, 0) == Approx(-1.0));
        REQUIRE(error(5, 0) == Approx(0.0));


        error = cost_function.getError({0.0, M_PI / 4.0, 0.0});

        REQUIRE(error(0, 0) == Approx(0.0));
        REQUIRE(error(1, 0) == Approx(0.0));
        REQUIRE(error(2, 0) == Approx(-0.785398));
        REQUIRE(error(3, 0) == Approx(0.707107));
        REQUIRE(error(4, 0) == Approx(-0.292893));
        REQUIRE(error(5, 0) == Approx(0.0));


        error = cost_function.getError({0.0, M_PI / 2.0, 0.0});

        REQUIRE(error(0, 0) == Approx(0.0).margin(1e-4));
        REQUIRE(error(1, 0) == Approx(0.0).margin(1e-4));
        REQUIRE(error(2, 0) == Approx(0.0).margin(1e-4));
        REQUIRE(error(3, 0) == Approx(0.0).margin(1e-4));
        REQUIRE(error(4, 0) == Approx(0.0).margin(1e-4));
        REQUIRE(error(5, 0) == Approx(0.0).margin(1e-4));
    }

    SECTION("Get gradient and hessian")
    {
        Eigen::Matrix<double, 3, 1> gradient;
        Eigen::Matrix<double, 3, 3> hessian;

        cost_function.getGradientAndHessian({0.0, 0.0, 0.0}, gradient, hessian);

        REQUIRE(gradient(0, 0) == Approx(-3.5708));
        REQUIRE(gradient(1, 0) == Approx(-2.5708));
        REQUIRE(gradient(2, 0) == Approx(-1.5708));

        REQUIRE(hessian(0, 0) == Approx(5.0));
        REQUIRE(hessian(1, 0) == Approx(3.0));
        REQUIRE(hessian(2, 0) == Approx(1.0));

        REQUIRE(hessian(0, 1) == Approx(3.0));
        REQUIRE(hessian(1, 1) == Approx(2.0));
        REQUIRE(hessian(2, 1) == Approx(1.0));

        REQUIRE(hessian(0, 2) == Approx(1.0));
        REQUIRE(hessian(1, 2) == Approx(1.0));
        REQUIRE(hessian(2, 2) == Approx(1.0));


        cost_function.getGradientAndHessian({0.0, M_PI / 4.0, 0.0}, gradient, hessian);

        REQUIRE(gradient(0, 0) == Approx(-1.7854));
        REQUIRE(gradient(1, 0) == Approx(-1.4925));
        REQUIRE(gradient(2, 0) == Approx(-0.785398));

        REQUIRE(hessian(0, 0) == Approx(4.41421));
        REQUIRE(hessian(1, 0) == Approx(2.70711));
        REQUIRE(hessian(2, 0) == Approx(1.0));

        REQUIRE(hessian(0, 1) == Approx(2.70711));
        REQUIRE(hessian(1, 1) == Approx(2.0));
        REQUIRE(hessian(2, 1) == Approx(1.0).margin(1e-3));

        REQUIRE(hessian(0, 2) == Approx(1.0));
        REQUIRE(hessian(1, 2) == Approx(1.0));
        REQUIRE(hessian(2, 2) == Approx(1.0));


        cost_function.getGradientAndHessian({0.0, M_PI / 2.0, 0.0}, gradient, hessian);

        REQUIRE(gradient(0, 0) == Approx(0.0).margin(1e-3));
        REQUIRE(gradient(1, 0) == Approx(0.0).margin(1e-3));
        REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));

        REQUIRE(hessian(0, 0) == Approx(2.0));
        REQUIRE(hessian(1, 0) == Approx(1.0));
        REQUIRE(hessian(2, 0) == Approx(0.0).margin(1e-3));

        REQUIRE(hessian(0, 1) == Approx(1.0));
        REQUIRE(hessian(1, 1) == Approx(1.0));
        REQUIRE(hessian(2, 1) == Approx(0.0).margin(1e-3));

        REQUIRE(hessian(0, 2) == Approx(0.0).margin(1e-3));
        REQUIRE(hessian(1, 2) == Approx(0.0).margin(1e-3));
        REQUIRE(hessian(2, 2) == Approx(0.0).margin(1e-3));
    }

}
