#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE("SingleManipulatorTarget", "[SingleManipulatorTarget]")
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

    SECTION("Point tool, point target")
    {
        Eigen::Vector<double, 3> target_position({ 0.0, M_PI / 2.0, 0.0 });

        gafro::Point<double> ee_target_point = manipulator.getEEMotor(target_position).apply(gafro::Point<double>());

        SingleManipulatorTarget<double, 3, Point, Point> cost_function(&manipulator, gafro::Point<double>(), ee_target_point);

        SECTION("Get value")
        {
            REQUIRE(cost_function.getValue({0.0, 0.0, 0.0}) == Approx(37.5));
            REQUIRE(cost_function.getValue({0.2, 0.0, 0.0}) == Approx(24.145).margin(1e-3));
            REQUIRE(cost_function.getValue({0.0, M_PI / 4.0, 0.0}) == Approx(9.989).margin(1e-3));
            REQUIRE(cost_function.getValue({0.0, M_PI / 2.0, 0.0}) == Approx(0.0));
        }

        SECTION("Get error")
        {
            auto error = cost_function.getError({0.0, 0.0, 0.0});

            REQUIRE(error.rows() == 10);
            REQUIRE(error(0, 0) == Approx(0.0));
            REQUIRE(error(1, 0) == Approx(0.0));
            REQUIRE(error(2, 0) == Approx(-3.0));
            REQUIRE(error(3, 0) == Approx(-4.5));
            REQUIRE(error(4, 0) == Approx(1.5));
            REQUIRE(error(5, 0) == Approx(0.0));
            REQUIRE(error(6, 0) == Approx(1.0));
            REQUIRE(error(7, 0) == Approx(1.0));
            REQUIRE(error(8, 0) == Approx(0.0));
            REQUIRE(error(9, 0) == Approx(2.0));


            error = cost_function.getError({0.0, M_PI / 4.0, 0.0});

            REQUIRE(error(0, 0) == Approx(0.0));
            REQUIRE(error(1, 0) == Approx(0.0));
            REQUIRE(error(2, 0) == Approx(-1.29289));
            REQUIRE(error(3, 0) == Approx(-2.14645));
            REQUIRE(error(4, 0) == Approx(1.06066));
            REQUIRE(error(5, 0) == Approx(0.0));
            REQUIRE(error(6, 0) == Approx(0.292893));
            REQUIRE(error(7, 0) == Approx(0.707107));
            REQUIRE(error(8, 0) == Approx(0.0));
            REQUIRE(error(9, 0) == Approx(1.41421));


            error = cost_function.getError({0.0, M_PI / 2.0, 0.0});

            REQUIRE(error(0, 0) == Approx(0.0));
            REQUIRE(error(1, 0) == Approx(0.0));
            REQUIRE(error(2, 0) == Approx(0.0));
            REQUIRE(error(3, 0) == Approx(0.0));
            REQUIRE(error(4, 0) == Approx(0.0));
            REQUIRE(error(5, 0) == Approx(0.0));
            REQUIRE(error(6, 0) == Approx(0.0));
            REQUIRE(error(7, 0) == Approx(0.0));
            REQUIRE(error(8, 0) == Approx(0.0));
            REQUIRE(error(9, 0) == Approx(0.0));
        }

        SECTION("Get jacobian")
        {
            auto jacobian = cost_function.getJacobian({0.0, 0.0, 0.0});

            REQUIRE(jacobian.rows() == 10);
            REQUIRE(jacobian.cols() == 3);

            REQUIRE(jacobian(0, 0) == Approx(0.0));
            REQUIRE(jacobian(1, 0) == Approx(0.0));
            REQUIRE(jacobian(2, 0) == Approx(8.0));
            REQUIRE(jacobian(3, 0) == Approx(10.0));
            REQUIRE(jacobian(4, 0) == Approx(0.0));
            REQUIRE(jacobian(5, 0) == Approx(0.0));
            REQUIRE(jacobian(6, 0) == Approx(-4.0));
            REQUIRE(jacobian(7, 0) == Approx(0.0));
            REQUIRE(jacobian(8, 0) == Approx(0.0));
            REQUIRE(jacobian(9, 0) == Approx(0.0));

            REQUIRE(jacobian(0, 1) == Approx(0.0));
            REQUIRE(jacobian(1, 1) == Approx(0.0));
            REQUIRE(jacobian(2, 1) == Approx(4.0));
            REQUIRE(jacobian(3, 1) == Approx(5.0));
            REQUIRE(jacobian(4, 1) == Approx(0.0));
            REQUIRE(jacobian(5, 1) == Approx(0.0));
            REQUIRE(jacobian(6, 1) == Approx(-2.0));
            REQUIRE(jacobian(7, 1) == Approx(0.0));
            REQUIRE(jacobian(8, 1) == Approx(0.0));
            REQUIRE(jacobian(9, 1) == Approx(0.0));

            REQUIRE(jacobian(0, 2) == Approx(0.0));
            REQUIRE(jacobian(1, 2) == Approx(0.0));
            REQUIRE(jacobian(2, 2) == Approx(0.0));
            REQUIRE(jacobian(3, 2) == Approx(0.0));
            REQUIRE(jacobian(4, 2) == Approx(0.0));
            REQUIRE(jacobian(5, 2) == Approx(0.0));
            REQUIRE(jacobian(6, 2) == Approx(0.0));
            REQUIRE(jacobian(7, 2) == Approx(0.0));
            REQUIRE(jacobian(8, 2) == Approx(0.0));
            REQUIRE(jacobian(9, 2) == Approx(0.0));


            jacobian = cost_function.getJacobian({0.0, M_PI / 4.0, 0.0});

            REQUIRE(jacobian(0, 0) == Approx(0.0));
            REQUIRE(jacobian(1, 0) == Approx(0.0));
            REQUIRE(jacobian(2, 0) == Approx(8.24264));
            REQUIRE(jacobian(3, 0) == Approx(9.94975));
            REQUIRE(jacobian(4, 0) == Approx(0.707107));
            REQUIRE(jacobian(5, 0) == Approx(0.0));
            REQUIRE(jacobian(6, 0) == Approx(-3.41421));
            REQUIRE(jacobian(7, 0) == Approx(-1.41421));
            REQUIRE(jacobian(8, 0) == Approx(0.0));
            REQUIRE(jacobian(9, 0) == Approx(-1.41421));

            REQUIRE(jacobian(0, 1) == Approx(0.0));
            REQUIRE(jacobian(1, 1) == Approx(0.0));
            REQUIRE(jacobian(2, 1) == Approx(4.24264));
            REQUIRE(jacobian(3, 1) == Approx(6.36396));
            REQUIRE(jacobian(4, 1) == Approx(-2.12132));
            REQUIRE(jacobian(5, 1) == Approx(0.0));
            REQUIRE(jacobian(6, 1) == Approx(-1.41421));
            REQUIRE(jacobian(7, 1) == Approx(-1.41421));
            REQUIRE(jacobian(8, 1) == Approx(0.0));
            REQUIRE(jacobian(9, 1) == Approx(-2.82843));

            REQUIRE(jacobian(0, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(1, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(2, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(3, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(4, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(5, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(6, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(7, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(8, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(9, 2) == Approx(0.0).margin(1e-3));


            jacobian = cost_function.getJacobian({0.0, M_PI / 2.0, 0.0});

            REQUIRE(jacobian(0, 0) == Approx(0.0));
            REQUIRE(jacobian(1, 0) == Approx(0.0));
            REQUIRE(jacobian(2, 0) == Approx(6.0));
            REQUIRE(jacobian(3, 0) == Approx(7.0));
            REQUIRE(jacobian(4, 0) == Approx(1.0));
            REQUIRE(jacobian(5, 0) == Approx(0.0));
            REQUIRE(jacobian(6, 0) == Approx(-2.0));
            REQUIRE(jacobian(7, 0) == Approx(-2.0));
            REQUIRE(jacobian(8, 0) == Approx(0.0));
            REQUIRE(jacobian(9, 0) == Approx(-2.0));

            REQUIRE(jacobian(0, 1) == Approx(0.0));
            REQUIRE(jacobian(1, 1) == Approx(0.0));
            REQUIRE(jacobian(2, 1) == Approx(2.0));
            REQUIRE(jacobian(3, 1) == Approx(4.0));
            REQUIRE(jacobian(4, 1) == Approx(-3.0));
            REQUIRE(jacobian(5, 1) == Approx(0.0));
            REQUIRE(jacobian(6, 1) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(7, 1) == Approx(-2.0));
            REQUIRE(jacobian(8, 1) == Approx(0.0));
            REQUIRE(jacobian(9, 1) == Approx(-4.0));

            REQUIRE(jacobian(0, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(1, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(2, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(3, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(4, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(5, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(6, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(7, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(8, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(9, 2) == Approx(0.0).margin(1e-3));
        }

        SECTION("Get gradient")
        {
            auto gradient = cost_function.getGradient({0.0, 0.0, 0.0});

            REQUIRE(gradient.rows() == 3);
            REQUIRE(gradient.cols() == 1);

            REQUIRE(gradient(0, 0) == Approx(-73.0));
            REQUIRE(gradient(1, 0) == Approx(-36.5));
            REQUIRE(gradient(2, 0) == Approx(0.0));

            gradient = cost_function.getGradient({0.0, M_PI / 4.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(-35.2635));
            REQUIRE(gradient(1, 0) == Approx(-26.8094));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));

            gradient = cost_function.getGradient({0.0, M_PI / 2.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(0.0));
            REQUIRE(gradient(1, 0) == Approx(0.0));
            REQUIRE(gradient(2, 0) == Approx(0.0));
        }

        SECTION("Get gradient and hessian")
        {
            Eigen::Matrix<double, 3, 1> gradient;
            Eigen::Matrix<double, 3, 3> hessian;

            cost_function.getGradientAndHessian({0.0, 0.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(-73.0));
            REQUIRE(gradient(1, 0) == Approx(-36.5));
            REQUIRE(gradient(2, 0) == Approx(0.0));

            REQUIRE(hessian(0, 0) == Approx(180.0));
            REQUIRE(hessian(1, 0) == Approx(90.0));
            REQUIRE(hessian(2, 0) == Approx(0.0));

            REQUIRE(hessian(0, 1) == Approx(90.0));
            REQUIRE(hessian(1, 1) == Approx(45.0));
            REQUIRE(hessian(2, 1) == Approx(0.0));

            REQUIRE(hessian(0, 2) == Approx(0.0));
            REQUIRE(hessian(1, 2) == Approx(0.0));
            REQUIRE(hessian(2, 2) == Approx(0.0));


            cost_function.getGradientAndHessian({0.0, M_PI / 4.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(-35.2635));
            REQUIRE(gradient(1, 0) == Approx(-26.8094));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 0) == Approx(183.095));
            REQUIRE(hessian(1, 0) == Approx(107.619));
            REQUIRE(hessian(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 1) == Approx(107.619));
            REQUIRE(hessian(1, 1) == Approx(75.0));
            REQUIRE(hessian(2, 1) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(1, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(2, 2) == Approx(0.0).margin(1e-3));


            cost_function.getGradientAndHessian({0.0, M_PI / 2.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(0.0));
            REQUIRE(gradient(1, 0) == Approx(0.0));
            REQUIRE(gradient(2, 0) == Approx(0.0));

            REQUIRE(hessian(0, 0) == Approx(98.0));
            REQUIRE(hessian(1, 0) == Approx(49.0));
            REQUIRE(hessian(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 1) == Approx(49.0));
            REQUIRE(hessian(1, 1) == Approx(49.0));
            REQUIRE(hessian(2, 1) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(1, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(2, 2) == Approx(0.0).margin(1e-3));
        }
    }

    SECTION("Point tool with offset, point target")
    {
        Eigen::Vector<double, 3> target_position({ 0.0, M_PI / 2.0, 0.0 });

        gafro::Point<double> ee_target_point = manipulator.getEEMotor(target_position).apply(gafro::Point<double>());

        SingleManipulatorTarget<double, 3, Point, Point> cost_function(&manipulator, gafro::Point<double>(0.0, 0.1, 0.0), ee_target_point);

        SECTION("Get value")
        {
            REQUIRE(cost_function.getValue({0.0, 0.0, 0.0}) == Approx(43.68065));
            REQUIRE(cost_function.getValue({0.2, 0.0, 0.0}) == Approx(28.903).margin(1e-3));
            REQUIRE(cost_function.getValue({0.0, M_PI / 4.0, 0.0}) == Approx(11.712).margin(1e-3));
            REQUIRE(cost_function.getValue({0.0, M_PI / 2.0, 0.0}) == Approx(0.12615));
        }

        SECTION("Get error")
        {
            auto error = cost_function.getError({0.0, 0.0, 0.0});

            REQUIRE(error.rows() == 10);
            REQUIRE(error(0, 0) == Approx(0.0));
            REQUIRE(error(1, 0) == Approx(0.0));
            REQUIRE(error(2, 0) == Approx(-3.1));
            REQUIRE(error(3, 0) == Approx(-4.805));
            REQUIRE(error(4, 0) == Approx(1.86));
            REQUIRE(error(5, 0) == Approx(0.0));
            REQUIRE(error(6, 0) == Approx(1.0));
            REQUIRE(error(7, 0) == Approx(1.1));
            REQUIRE(error(8, 0) == Approx(0.0));
            REQUIRE(error(9, 0) == Approx(2.305));


            error = cost_function.getError({0.0, M_PI / 4.0, 0.0});

            REQUIRE(error(0, 0) == Approx(0.0));
            REQUIRE(error(1, 0) == Approx(0.0));
            REQUIRE(error(2, 0) == Approx(-1.22218));
            REQUIRE(error(3, 0) == Approx(-2.21609));
            REQUIRE(error(4, 0) == Approx(1.37673));
            REQUIRE(error(5, 0) == Approx(0.0));
            REQUIRE(error(6, 0) == Approx(0.222183));
            REQUIRE(error(7, 0) == Approx(0.777817));
            REQUIRE(error(8, 0) == Approx(0.0));
            REQUIRE(error(9, 0) == Approx(1.66063));


            error = cost_function.getError({0.0, M_PI / 2.0, 0.0});

            REQUIRE(error(0, 0) == Approx(0.0));
            REQUIRE(error(1, 0) == Approx(0.0));
            REQUIRE(error(2, 0) == Approx(0.2));
            REQUIRE(error(3, 0) == Approx(0.145));
            REQUIRE(error(4, 0) == Approx(0.21));
            REQUIRE(error(5, 0) == Approx(0.0));
            REQUIRE(error(6, 0) == Approx(-0.1));
            REQUIRE(error(7, 0) == Approx(0.0).margin(1e-3));
            REQUIRE(error(8, 0) == Approx(0.0));
            REQUIRE(error(9, 0) == Approx(0.105));
        }

        SECTION("Get jacobian")
        {
            auto jacobian = cost_function.getJacobian({0.0, 0.0, 0.0});

            REQUIRE(jacobian.rows() == 10);
            REQUIRE(jacobian.cols() == 3);

            REQUIRE(jacobian(0, 0) == Approx(0.0));
            REQUIRE(jacobian(1, 0) == Approx(0.0));
            REQUIRE(jacobian(2, 0) == Approx(8.4));
            REQUIRE(jacobian(3, 0) == Approx(10.5));
            REQUIRE(jacobian(4, 0) == Approx(0.0));
            REQUIRE(jacobian(5, 0) == Approx(0.0));
            REQUIRE(jacobian(6, 0) == Approx(-4.2));
            REQUIRE(jacobian(7, 0) == Approx(0.0));
            REQUIRE(jacobian(8, 0) == Approx(0.0));
            REQUIRE(jacobian(9, 0) == Approx(0.0));

            REQUIRE(jacobian(0, 1) == Approx(0.0));
            REQUIRE(jacobian(1, 1) == Approx(0.0));
            REQUIRE(jacobian(2, 1) == Approx(4.4));
            REQUIRE(jacobian(3, 1) == Approx(5.5));
            REQUIRE(jacobian(4, 1) == Approx(0.0));
            REQUIRE(jacobian(5, 1) == Approx(0.0));
            REQUIRE(jacobian(6, 1) == Approx(-2.2));
            REQUIRE(jacobian(7, 1) == Approx(0.0));
            REQUIRE(jacobian(8, 1) == Approx(0.0));
            REQUIRE(jacobian(9, 1) == Approx(0.0));

            REQUIRE(jacobian(0, 2) == Approx(0.0));
            REQUIRE(jacobian(1, 2) == Approx(0.0));
            REQUIRE(jacobian(2, 2) == Approx(0.4));
            REQUIRE(jacobian(3, 2) == Approx(0.5));
            REQUIRE(jacobian(4, 2) == Approx(0.0));
            REQUIRE(jacobian(5, 2) == Approx(0.0));
            REQUIRE(jacobian(6, 2) == Approx(-0.2));
            REQUIRE(jacobian(7, 2) == Approx(0.0));
            REQUIRE(jacobian(8, 2) == Approx(0.0));
            REQUIRE(jacobian(9, 2) == Approx(0.0));


            jacobian = cost_function.getJacobian({0.0, M_PI / 4.0, 0.0});

            REQUIRE(jacobian(0, 0) == Approx(0.0));
            REQUIRE(jacobian(1, 0) == Approx(0.0));
            REQUIRE(jacobian(2, 0) == Approx(8.6669));
            REQUIRE(jacobian(3, 0) == Approx(10.4447));
            REQUIRE(jacobian(4, 0) == Approx(0.777817));
            REQUIRE(jacobian(5, 0) == Approx(0.0));
            REQUIRE(jacobian(6, 0) == Approx(-3.55563));
            REQUIRE(jacobian(7, 0) == Approx(-1.55563));
            REQUIRE(jacobian(8, 0) == Approx(0.0));
            REQUIRE(jacobian(9, 0) == Approx(-1.55563));

            REQUIRE(jacobian(0, 1) == Approx(0.0));
            REQUIRE(jacobian(1, 1) == Approx(0.0));
            REQUIRE(jacobian(2, 1) == Approx(4.6669));
            REQUIRE(jacobian(3, 1) == Approx(7.00036));
            REQUIRE(jacobian(4, 1) == Approx(-2.33345));
            REQUIRE(jacobian(5, 1) == Approx(0.0));
            REQUIRE(jacobian(6, 1) == Approx(-1.55563));
            REQUIRE(jacobian(7, 1) == Approx(-1.55563));
            REQUIRE(jacobian(8, 1) == Approx(0.0));
            REQUIRE(jacobian(9, 1) == Approx(-3.11127));

            REQUIRE(jacobian(0, 2) == Approx(0.0));
            REQUIRE(jacobian(1, 2) == Approx(0.0));
            REQUIRE(jacobian(2, 2) == Approx(0.424264));
            REQUIRE(jacobian(3, 2) == Approx(0.636396));
            REQUIRE(jacobian(4, 2) == Approx(-0.212132));
            REQUIRE(jacobian(5, 2) == Approx(0.0));
            REQUIRE(jacobian(6, 2) == Approx(-0.141421));
            REQUIRE(jacobian(7, 2) == Approx(-0.141421));
            REQUIRE(jacobian(8, 2) == Approx(0.0));
            REQUIRE(jacobian(9, 2) == Approx(-0.282843));


            jacobian = cost_function.getJacobian({0.0, M_PI / 2.0, 0.0});

            REQUIRE(jacobian(0, 0) == Approx(0.0));
            REQUIRE(jacobian(1, 0) == Approx(0.0));
            REQUIRE(jacobian(2, 0) == Approx(6.2));
            REQUIRE(jacobian(3, 0) == Approx(7.2));
            REQUIRE(jacobian(4, 0) == Approx(1.1));
            REQUIRE(jacobian(5, 0) == Approx(0.0));
            REQUIRE(jacobian(6, 0) == Approx(-2.0));
            REQUIRE(jacobian(7, 0) == Approx(-2.2));
            REQUIRE(jacobian(8, 0) == Approx(0.0));
            REQUIRE(jacobian(9, 0) == Approx(-2.2));

            REQUIRE(jacobian(0, 1) == Approx(0.0));
            REQUIRE(jacobian(1, 1) == Approx(0.0));
            REQUIRE(jacobian(2, 1) == Approx(2.2));
            REQUIRE(jacobian(3, 1) == Approx(4.4));
            REQUIRE(jacobian(4, 1) == Approx(-3.3));
            REQUIRE(jacobian(5, 1) == Approx(0.0));
            REQUIRE(jacobian(6, 1) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(7, 1) == Approx(-2.2));
            REQUIRE(jacobian(8, 1) == Approx(0.0));
            REQUIRE(jacobian(9, 1) == Approx(-4.4));

            REQUIRE(jacobian(0, 2) == Approx(0.0));
            REQUIRE(jacobian(1, 2) == Approx(0.0));
            REQUIRE(jacobian(2, 2) == Approx(0.2));
            REQUIRE(jacobian(3, 2) == Approx(0.4));
            REQUIRE(jacobian(4, 2) == Approx(-0.3));
            REQUIRE(jacobian(5, 2) == Approx(0.0));
            REQUIRE(jacobian(6, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(jacobian(7, 2) == Approx(-0.2));
            REQUIRE(jacobian(8, 2) == Approx(0.0));
            REQUIRE(jacobian(9, 2) == Approx(-0.4));
        }

        SECTION("Get gradient")
        {
            auto gradient = cost_function.getGradient({0.0, 0.0, 0.0});

            REQUIRE(gradient.rows() == 3);
            REQUIRE(gradient.cols() == 1);

            REQUIRE(gradient(0, 0) == Approx(-80.6925));
            REQUIRE(gradient(1, 0) == Approx(-42.2675));
            REQUIRE(gradient(2, 0) == Approx(-3.8425));

            gradient = cost_function.getGradient({0.0, M_PI / 4.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(-37.2515));
            REQUIRE(gradient(1, 0) == Approx(-31.1521));
            REQUIRE(gradient(2, 0) == Approx(-2.83201));

            gradient = cost_function.getGradient({0.0, M_PI / 2.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(2.484));
            REQUIRE(gradient(1, 0) == Approx(-0.077));
            REQUIRE(gradient(2, 0) == Approx(-0.007));
        }

        SECTION("Get gradient and hessian")
        {
            Eigen::Matrix<double, 3, 1> gradient;
            Eigen::Matrix<double, 3, 3> hessian;

            cost_function.getGradientAndHessian({0.0, 0.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(-80.6925));
            REQUIRE(gradient(1, 0) == Approx(-42.2675));
            REQUIRE(gradient(2, 0) == Approx(-3.8425));

            REQUIRE(hessian(0, 0) == Approx(198.45));
            REQUIRE(hessian(1, 0) == Approx(103.95));
            REQUIRE(hessian(2, 0) == Approx(9.45));

            REQUIRE(hessian(0, 1) == Approx(103.95));
            REQUIRE(hessian(1, 1) == Approx(54.45));
            REQUIRE(hessian(2, 1) == Approx(4.95));

            REQUIRE(hessian(0, 2) == Approx(9.45));
            REQUIRE(hessian(1, 2) == Approx(4.95));
            REQUIRE(hessian(2, 2) == Approx(0.45));


            cost_function.getGradientAndHessian({0.0, M_PI / 4.0, 0.0}, gradient, hessian);
 
            REQUIRE(gradient(0, 0) == Approx(-37.2515));
            REQUIRE(gradient(1, 0) == Approx(-31.1521));
            REQUIRE(gradient(2, 0) == Approx(-2.83201));

            REQUIRE(hessian(0, 0) == Approx(202.295));
            REQUIRE(hessian(1, 0) == Approx(124.541));
            REQUIRE(hessian(2, 0) == Approx(11.3219));

            REQUIRE(hessian(0, 1) == Approx(124.541));
            REQUIRE(hessian(1, 1) == Approx(90.75));
            REQUIRE(hessian(2, 1) == Approx(8.25));

            REQUIRE(hessian(0, 2) == Approx(11.3219));
            REQUIRE(hessian(1, 2) == Approx(8.25));
            REQUIRE(hessian(2, 2) == Approx(0.75));


            cost_function.getGradientAndHessian({0.0, M_PI / 2.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(2.484));
            REQUIRE(gradient(1, 0) == Approx(-0.077));
            REQUIRE(gradient(2, 0) == Approx(-0.007));

            REQUIRE(hessian(0, 0) == Approx(105.17));
            REQUIRE(hessian(1, 0) == Approx(56.21));
            REQUIRE(hessian(2, 0) == Approx(5.11));

            REQUIRE(hessian(0, 1) == Approx(56.21));
            REQUIRE(hessian(1, 1) == Approx(59.29));
            REQUIRE(hessian(2, 1) == Approx(5.39));

            REQUIRE(hessian(0, 2) == Approx(5.11));
            REQUIRE(hessian(1, 2) == Approx(5.39));
            REQUIRE(hessian(2, 2) == Approx(0.49));
        }
    }

    SECTION("Point tool, sphere target")
    {
        Eigen::Vector<double, 3> target_position({ 0.0, M_PI / 2.0, 0.0 });

        Sphere<double> ee_target_sphere = manipulator.getEEMotor(target_position).apply(Sphere<double>(Point<double>(), 0.1));

        SingleManipulatorTarget<double, 3, Point, Sphere> cost_function(&manipulator, Point<double>(), ee_target_sphere);

        SECTION("Get value")
        {
            REQUIRE(cost_function.getValue({0.0, 0.0, 0.0}) == Approx(0.990025));
            REQUIRE(cost_function.getValue({0.2, 0.0, 0.0}) == Approx(0.4064).margin(1e-4));
            REQUIRE(cost_function.getValue({0.0, M_PI / 4.0, 0.0}) == Approx(0.0828).margin(1e-4));
            REQUIRE(cost_function.getValue({0.0, M_PI / 2.0, 0.0}) == Approx(0.000025));
        }

        SECTION("Get error")
        {
            auto error = cost_function.getError({0.0, 0.0, 0.0});

            REQUIRE(error.rows() == 1);
            REQUIRE(error(0, 0) == Approx(-0.995));

            error = cost_function.getError({0.0, M_PI / 4.0, 0.0});
            REQUIRE(error(0, 0) == Approx(-0.287893));

            error = cost_function.getError({0.0, M_PI / 2.0, 0.0});
            REQUIRE(error(0, 0) == Approx(0.005));
        }

        SECTION("Get jacobian")
        {
            auto jacobian = cost_function.getJacobian({0.0, 0.0, 0.0});

            REQUIRE(jacobian.rows() == 1);
            REQUIRE(jacobian.cols() == 3);

            REQUIRE(jacobian(0, 0) == Approx(4.0));
            REQUIRE(jacobian(0, 1) == Approx(2.0));
            REQUIRE(jacobian(0, 2) == Approx(0.0));


            jacobian = cost_function.getJacobian({0.0, M_PI / 4.0, 0.0});

            REQUIRE(jacobian(0, 0) == Approx(2.0));
            REQUIRE(jacobian(0, 1) == Approx(1.41421));
            REQUIRE(jacobian(0, 2) == Approx(0.0).margin(1e-3));


            jacobian = cost_function.getJacobian({0.0, M_PI / 2.0, 0.0});

            REQUIRE(jacobian(0, 0) == Approx(0.0));
            REQUIRE(jacobian(0, 1) == Approx(0.0));
            REQUIRE(jacobian(0, 2) == Approx(0.0).margin(1e-3));
        }

        SECTION("Get gradient")
        {
            auto gradient = cost_function.getGradient({0.0, 0.0, 0.0});

            REQUIRE(gradient.rows() == 3);
            REQUIRE(gradient.cols() == 1);

            REQUIRE(gradient(0, 0) == Approx(-3.98));
            REQUIRE(gradient(1, 0) == Approx(-1.99));
            REQUIRE(gradient(2, 0) == Approx(0.0));

            gradient = cost_function.getGradient({0.0, M_PI / 4.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(-0.575786));
            REQUIRE(gradient(1, 0) == Approx(-0.407142));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));

            gradient = cost_function.getGradient({0.0, M_PI / 2.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(0.0));
            REQUIRE(gradient(1, 0) == Approx(0.0));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));
        }

        SECTION("Get gradient and hessian")
        {
            Eigen::Matrix<double, 3, 1> gradient;
            Eigen::Matrix<double, 3, 3> hessian;

            cost_function.getGradientAndHessian({0.0, 0.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(-3.98));
            REQUIRE(gradient(1, 0) == Approx(-1.99));
            REQUIRE(gradient(2, 0) == Approx(0.0));

            REQUIRE(hessian(0, 0) == Approx(16.0));
            REQUIRE(hessian(1, 0) == Approx(8.0));
            REQUIRE(hessian(2, 0) == Approx(0.0));

            REQUIRE(hessian(0, 1) == Approx(8.0));
            REQUIRE(hessian(1, 1) == Approx(4.0));
            REQUIRE(hessian(2, 1) == Approx(0.0));

            REQUIRE(hessian(0, 2) == Approx(0.0));
            REQUIRE(hessian(1, 2) == Approx(0.0));
            REQUIRE(hessian(2, 2) == Approx(0.0));


            cost_function.getGradientAndHessian({0.0, M_PI / 4.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(-0.575786));
            REQUIRE(gradient(1, 0) == Approx(-0.407142));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 0) == Approx(4.0));
            REQUIRE(hessian(1, 0) == Approx(2.82843));
            REQUIRE(hessian(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 1) == Approx(2.82843));
            REQUIRE(hessian(1, 1) == Approx(2.0));
            REQUIRE(hessian(2, 1) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(1, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(2, 2) == Approx(0.0).margin(1e-3));


            cost_function.getGradientAndHessian({0.0, M_PI / 2.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(0.0));
            REQUIRE(gradient(1, 0) == Approx(0.0));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 0) == Approx(0.0));
            REQUIRE(hessian(1, 0) == Approx(0.0));
            REQUIRE(hessian(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 1) == Approx(0.0));
            REQUIRE(hessian(1, 1) == Approx(0.0));
            REQUIRE(hessian(2, 1) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(1, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(2, 2) == Approx(0.0).margin(1e-3));
        }
    }

}
