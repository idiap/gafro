#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE("SingleManipulatorDualTarget", "[SingleManipulatorDualTarget]")
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

        SingleManipulatorDualTarget<double, 3, Point, Point> cost_function(&manipulator, gafro::Point<double>(), ee_target_point);

        SECTION("Get value")
        {
            REQUIRE(cost_function.getValue({0.0, 0.0, 0.0}) == Approx(1.0));
            REQUIRE(cost_function.getValue({0.2, 0.0, 0.0}) == Approx(0.41284).margin(1e-4));
            REQUIRE(cost_function.getValue({0.0, M_PI / 4.0, 0.0}) == Approx(0.0857).margin(1e-4));
            REQUIRE(cost_function.getValue({0.0, M_PI / 2.0, 0.0}) == Approx(0.0).margin(1e-4));
        }

        SECTION("Get error")
        {
            auto error = cost_function.getError({0.0, 0.0, 0.0});

            REQUIRE(error.rows() == 1);
            REQUIRE(error(0, 0) == Approx(1.0));

            error = cost_function.getError({0.0, M_PI / 4.0, 0.0});
            REQUIRE(error(0, 0) == Approx(0.2928).margin(1e-4));

            error = cost_function.getError({0.0, M_PI / 2.0, 0.0});
            REQUIRE(error(0, 0) == Approx(0.0).margin(1e-4));
        }

        SECTION("Get jacobian")
        {
            auto jacobian = cost_function.getJacobian({0.0, 0.0, 0.0});

            REQUIRE(jacobian.rows() == 1);
            REQUIRE(jacobian.cols() == 3);

            REQUIRE(jacobian(0, 0) == Approx(-4.0));
            REQUIRE(jacobian(0, 1) == Approx(-2.0));
            REQUIRE(jacobian(0, 2) == Approx(0.0));


            jacobian = cost_function.getJacobian({0.0, M_PI / 4.0, 0.0});

            REQUIRE(jacobian(0, 0) == Approx(-2.0));
            REQUIRE(jacobian(0, 1) == Approx(-1.41421));
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

            REQUIRE(gradient(0, 0) == Approx(-4.0));
            REQUIRE(gradient(1, 0) == Approx(-2.0));
            REQUIRE(gradient(2, 0) == Approx(0.0));

            gradient = cost_function.getGradient({0.0, M_PI / 4.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(-0.585786));
            REQUIRE(gradient(1, 0) == Approx(-0.414214));
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

            REQUIRE(gradient(0, 0) == Approx(-4.0));
            REQUIRE(gradient(1, 0) == Approx(-2.0));
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

            REQUIRE(gradient(0, 0) == Approx(-0.585786));
            REQUIRE(gradient(1, 0) == Approx(-0.414214));
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

            REQUIRE(hessian(0, 0) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(1, 0) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 1) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(1, 1) == Approx(0.0).margin(1e-3));
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

        SingleManipulatorDualTarget<double, 3, Point, Point> cost_function(&manipulator, gafro::Point<double>(0.0, 0.1, 0.0), ee_target_point);

        SECTION("Get value")
        {
            REQUIRE(cost_function.getValue({0.0, 0.0, 0.0}) == Approx(1.221025));
            REQUIRE(cost_function.getValue({0.2, 0.0, 0.0}) == Approx(0.5323).margin(1e-4));
            REQUIRE(cost_function.getValue({0.0, M_PI / 4.0, 0.0}) == Approx(0.107).margin(1e-3));
            REQUIRE(cost_function.getValue({0.0, M_PI / 2.0, 0.0}) == Approx(0.000025));
        }

        SECTION("Get error")
        {
            auto error = cost_function.getError({0.0, 0.0, 0.0});

            REQUIRE(error.rows() == 1);
            REQUIRE(error(0, 0) == Approx(1.105));

            error = cost_function.getError({0.0, M_PI / 4.0, 0.0});
            REQUIRE(error(0, 0) == Approx(0.3271).margin(1e-4));

            error = cost_function.getError({0.0, M_PI / 2.0, 0.0});
            REQUIRE(error(0, 0) == Approx(0.005));
        }

        SECTION("Get jacobian")
        {
            auto jacobian = cost_function.getJacobian({0.0, 0.0, 0.0});

            REQUIRE(jacobian.rows() == 1);
            REQUIRE(jacobian.cols() == 3);

            REQUIRE(jacobian(0, 0) == Approx(-4.2));
            REQUIRE(jacobian(0, 1) == Approx(-2.2));
            REQUIRE(jacobian(0, 2) == Approx(-0.2));


            jacobian = cost_function.getJacobian({0.0, M_PI / 4.0, 0.0});

            REQUIRE(jacobian(0, 0) == Approx(-2.0));
            REQUIRE(jacobian(0, 1) == Approx(-1.55563));
            REQUIRE(jacobian(0, 2) == Approx(-0.141421));


            jacobian = cost_function.getJacobian({0.0, M_PI / 2.0, 0.0});

            REQUIRE(jacobian(0, 0) == Approx(0.2));
            REQUIRE(jacobian(0, 1) == Approx(0.0));
            REQUIRE(jacobian(0, 2) == Approx(0.0).margin(1e-3));
        }

        SECTION("Get gradient")
        {
            auto gradient = cost_function.getGradient({0.0, 0.0, 0.0});

            REQUIRE(gradient.rows() == 3);
            REQUIRE(gradient.cols() == 1);

            REQUIRE(gradient(0, 0) == Approx(-4.641));
            REQUIRE(gradient(1, 0) == Approx(-2.431));
            REQUIRE(gradient(2, 0) == Approx(-0.221));

            gradient = cost_function.getGradient({0.0, M_PI / 4.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(-0.654365));
            REQUIRE(gradient(1, 0) == Approx(-0.508977));
            REQUIRE(gradient(2, 0) == Approx(-0.0462706));

            gradient = cost_function.getGradient({0.0, M_PI / 2.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(0.001));
            REQUIRE(gradient(1, 0) == Approx(0.0));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));
        }

        SECTION("Get gradient and hessian")
        {
            Eigen::Matrix<double, 3, 1> gradient;
            Eigen::Matrix<double, 3, 3> hessian;

            cost_function.getGradientAndHessian({0.0, 0.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(-4.641));
            REQUIRE(gradient(1, 0) == Approx(-2.431));
            REQUIRE(gradient(2, 0) == Approx(-0.221));

            REQUIRE(hessian(0, 0) == Approx(17.64));
            REQUIRE(hessian(1, 0) == Approx(9.24));
            REQUIRE(hessian(2, 0) == Approx(0.84));

            REQUIRE(hessian(0, 1) == Approx(9.24));
            REQUIRE(hessian(1, 1) == Approx(4.84));
            REQUIRE(hessian(2, 1) == Approx(0.44));

            REQUIRE(hessian(0, 2) == Approx(0.84));
            REQUIRE(hessian(1, 2) == Approx(0.44));
            REQUIRE(hessian(2, 2) == Approx(0.04));


            cost_function.getGradientAndHessian({0.0, M_PI / 4.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(-0.654365));
            REQUIRE(gradient(1, 0) == Approx(-0.508977));
            REQUIRE(gradient(2, 0) == Approx(-0.0462706));

            REQUIRE(hessian(0, 0) == Approx(4.0));
            REQUIRE(hessian(1, 0) == Approx(3.11127));
            REQUIRE(hessian(2, 0) == Approx(0.282843));

            REQUIRE(hessian(0, 1) == Approx(3.11127));
            REQUIRE(hessian(1, 1) == Approx(2.42));
            REQUIRE(hessian(2, 1) == Approx(0.22));

            REQUIRE(hessian(0, 2) == Approx(0.282843));
            REQUIRE(hessian(1, 2) == Approx(0.22));
            REQUIRE(hessian(2, 2) == Approx(0.02));


            cost_function.getGradientAndHessian({0.0, M_PI / 2.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(0.001));
            REQUIRE(gradient(1, 0) == Approx(0.0));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 0) == Approx(0.04));
            REQUIRE(hessian(1, 0) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 1) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(1, 1) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(2, 1) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(1, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(2, 2) == Approx(0.0).margin(1e-3));
        }
    }

    SECTION("Point tool, sphere target")
    {
        Eigen::Vector<double, 3> target_position({ 0.0, M_PI / 2.0, 0.0 });

        Sphere<double> ee_target_sphere = manipulator.getEEMotor(target_position).apply(Sphere<double>(Point<double>(), 0.1));

        SingleManipulatorDualTarget<double, 3, Point, Sphere> cost_function(&manipulator, Point<double>(), ee_target_sphere);

        SECTION("Get value")
        {
            REQUIRE(cost_function.getValue({0.0, 0.0, 0.0}) == Approx(37.56525));
            REQUIRE(cost_function.getValue({0.2, 0.0, 0.0}) == Approx(24.2237).margin(1e-4));
            REQUIRE(cost_function.getValue({0.0, M_PI / 4.0, 0.0}) == Approx(10.0478).margin(1e-4));
            REQUIRE(cost_function.getValue({0.0, M_PI / 2.0, 0.0}) == Approx(0.00015));
        }

        SECTION("Get error")
        {
            auto error = cost_function.getError({0.0, 0.0, 0.0});

            REQUIRE(error.rows() == 10);
            REQUIRE(error(0, 0) == Approx(2.005));
            REQUIRE(error(1, 0) == Approx(0.0));
            REQUIRE(error(2, 0) == Approx(-1.515));
            REQUIRE(error(3, 0) == Approx(-4.5));
            REQUIRE(error(4, 0) == Approx(0.0));
            REQUIRE(error(5, 0) == Approx(-1.0));
            REQUIRE(error(6, 0) == Approx(1.0));
            REQUIRE(error(7, 0) == Approx(0.0));
            REQUIRE(error(8, 0) == Approx(0.0));
            REQUIRE(error(9, 0) == Approx(-3.0));

            error = cost_function.getError({0.0, M_PI / 4.0, 0.0});

            REQUIRE(error(0, 0) == Approx(1.41921));
            REQUIRE(error(1, 0) == Approx(0.0));
            REQUIRE(error(2, 0) == Approx(-1.0742));
            REQUIRE(error(3, 0) == Approx(-2.14998));
            REQUIRE(error(4, 0) == Approx(0.0));
            REQUIRE(error(5, 0) == Approx(-0.707107));
            REQUIRE(error(6, 0) == Approx(0.292893));
            REQUIRE(error(7, 0) == Approx(0.0));
            REQUIRE(error(8, 0) == Approx(0.0));
            REQUIRE(error(9, 0) == Approx(-1.29289));

            error = cost_function.getError({0.0, M_PI / 2.0, 0.0});

            REQUIRE(error(0, 0) == Approx(0.005));
            REQUIRE(error(1, 0) == Approx(0.0));
            REQUIRE(error(2, 0) == Approx(-0.01));
            REQUIRE(error(3, 0) == Approx(-0.005));
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
            REQUIRE(jacobian(2, 0) == Approx(0.0));
            REQUIRE(jacobian(3, 0) == Approx(9.98));
            REQUIRE(jacobian(4, 0) == Approx(0.0));
            REQUIRE(jacobian(5, 0) == Approx(0.0));
            REQUIRE(jacobian(6, 0) == Approx(-4.0));
            REQUIRE(jacobian(7, 0) == Approx(0.0));
            REQUIRE(jacobian(8, 0) == Approx(0.0));
            REQUIRE(jacobian(9, 0) == Approx(8.0));

            REQUIRE(jacobian(0, 1) == Approx(0.0));
            REQUIRE(jacobian(1, 1) == Approx(0.0));
            REQUIRE(jacobian(2, 1) == Approx(0.0));
            REQUIRE(jacobian(3, 1) == Approx(4.99));
            REQUIRE(jacobian(4, 1) == Approx(0.0));
            REQUIRE(jacobian(5, 1) == Approx(0.0));
            REQUIRE(jacobian(6, 1) == Approx(-2.0));
            REQUIRE(jacobian(7, 1) == Approx(0.0));
            REQUIRE(jacobian(8, 1) == Approx(0.0));
            REQUIRE(jacobian(9, 1) == Approx(4.0));

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

            REQUIRE(jacobian(0, 0) == Approx(-1.41421));
            REQUIRE(jacobian(1, 0) == Approx(0.0));
            REQUIRE(jacobian(2, 0) == Approx(-0.700036));
            REQUIRE(jacobian(3, 0) == Approx(9.93268));
            REQUIRE(jacobian(4, 0) == Approx(0.0));
            REQUIRE(jacobian(5, 0) == Approx(1.41421));
            REQUIRE(jacobian(6, 0) == Approx(-3.41421));
            REQUIRE(jacobian(7, 0) == Approx(0.0));
            REQUIRE(jacobian(8, 0) == Approx(0.0));
            REQUIRE(jacobian(9, 0) == Approx(8.24264));

            REQUIRE(jacobian(0, 1) == Approx(-2.82843));
            REQUIRE(jacobian(1, 1) == Approx(0.0));
            REQUIRE(jacobian(2, 1) == Approx(2.12839));
            REQUIRE(jacobian(3, 1) == Approx(6.35689));
            REQUIRE(jacobian(4, 1) == Approx(0.0));
            REQUIRE(jacobian(5, 1) == Approx(1.41421));
            REQUIRE(jacobian(6, 1) == Approx(-1.41421));
            REQUIRE(jacobian(7, 1) == Approx(0.0));
            REQUIRE(jacobian(8, 1) == Approx(0.0));
            REQUIRE(jacobian(9, 1) == Approx(4.24264));

            REQUIRE(jacobian(0, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(1, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(2, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(3, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(4, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(5, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(6, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(7, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(8, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(9, 2) == Approx(0.0).margin(1e-4));


            jacobian = cost_function.getJacobian({0.0, M_PI / 2.0, 0.0});

            REQUIRE(jacobian(0, 0) == Approx(-2.0));
            REQUIRE(jacobian(1, 0) == Approx(0.0));
            REQUIRE(jacobian(2, 0) == Approx(-0.99));
            REQUIRE(jacobian(3, 0) == Approx(6.99));
            REQUIRE(jacobian(4, 0) == Approx(0.0));
            REQUIRE(jacobian(5, 0) == Approx(2.0));
            REQUIRE(jacobian(6, 0) == Approx(-2.0));
            REQUIRE(jacobian(7, 0) == Approx(0.0));
            REQUIRE(jacobian(8, 0) == Approx(0.0));
            REQUIRE(jacobian(9, 0) == Approx(6.0));

            REQUIRE(jacobian(0, 1) == Approx(-4.0));
            REQUIRE(jacobian(1, 1) == Approx(0.0));
            REQUIRE(jacobian(2, 1) == Approx(3.01));
            REQUIRE(jacobian(3, 1) == Approx(4.0));
            REQUIRE(jacobian(4, 1) == Approx(0.0));
            REQUIRE(jacobian(5, 1) == Approx(2.0));
            REQUIRE(jacobian(6, 1) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(7, 1) == Approx(0.0));
            REQUIRE(jacobian(8, 1) == Approx(0.0));
            REQUIRE(jacobian(9, 1) == Approx(2.0));

            REQUIRE(jacobian(0, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(1, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(2, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(3, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(4, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(5, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(6, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(7, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(8, 2) == Approx(0.0).margin(1e-4));
            REQUIRE(jacobian(9, 2) == Approx(0.0).margin(1e-4));
        }

        SECTION("Get gradient")
        {
            auto gradient = cost_function.getGradient({0.0, 0.0, 0.0});

            REQUIRE(gradient.rows() == 3);
            REQUIRE(gradient.cols() == 1);

            REQUIRE(gradient(0, 0) == Approx(-72.91));
            REQUIRE(gradient(1, 0) == Approx(-36.455));
            REQUIRE(gradient(2, 0) == Approx(0.0));

            gradient = cost_function.getGradient({0.0, M_PI / 4.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(-35.267));
            REQUIRE(gradient(1, 0) == Approx(-26.8671));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));

            gradient = cost_function.getGradient({0.0, M_PI / 2.0, 0.0});

            REQUIRE(gradient(0, 0) == Approx(-0.03505));
            REQUIRE(gradient(1, 0) == Approx(-0.0701));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));
        }

        SECTION("Get gradient and hessian")
        {
            Eigen::Matrix<double, 3, 1> gradient;
            Eigen::Matrix<double, 3, 3> hessian;

            cost_function.getGradientAndHessian({0.0, 0.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(-72.91));
            REQUIRE(gradient(1, 0) == Approx(-36.455));
            REQUIRE(gradient(2, 0) == Approx(0.0));

            REQUIRE(hessian(0, 0) == Approx(179.6));
            REQUIRE(hessian(1, 0) == Approx(89.8002));
            REQUIRE(hessian(2, 0) == Approx(0.0));

            REQUIRE(hessian(0, 1) == Approx(89.8002));
            REQUIRE(hessian(1, 1) == Approx(44.9001));
            REQUIRE(hessian(2, 1) == Approx(0.0));

            REQUIRE(hessian(0, 2) == Approx(0.0));
            REQUIRE(hessian(1, 2) == Approx(0.0));
            REQUIRE(hessian(2, 2) == Approx(0.0));


            cost_function.getGradientAndHessian({0.0, M_PI / 4.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(-35.267));
            REQUIRE(gradient(1, 0) == Approx(-26.8671));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 0) == Approx(182.746));
            REQUIRE(hessian(1, 0) == Approx(107.45));
            REQUIRE(hessian(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 1) == Approx(107.45));
            REQUIRE(hessian(1, 1) == Approx(74.9401));
            REQUIRE(hessian(2, 1) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(1, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(2, 2) == Approx(0.0).margin(1e-3));


            cost_function.getGradientAndHessian({0.0, M_PI / 2.0, 0.0}, gradient, hessian);

            REQUIRE(gradient(0, 0) == Approx(-0.03505));
            REQUIRE(gradient(1, 0) == Approx(-0.0701));
            REQUIRE(gradient(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 0) == Approx(97.8402));
            REQUIRE(hessian(1, 0) == Approx(48.9801));
            REQUIRE(hessian(2, 0) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 1) == Approx(48.9801));
            REQUIRE(hessian(1, 1) == Approx(49.0601));
            REQUIRE(hessian(2, 1) == Approx(0.0).margin(1e-3));

            REQUIRE(hessian(0, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(1, 2) == Approx(0.0).margin(1e-3));
            REQUIRE(hessian(2, 2) == Approx(0.0).margin(1e-3));
        }
    }

}
