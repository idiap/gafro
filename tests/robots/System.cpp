#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;

TEST_CASE("System", "[System]")
{
    // Create some links
    std::unique_ptr<Link<double>> link1 = std::make_unique<Link<double>>();
    link1->setName("link1");

    std::unique_ptr<Link<double>> link2 = std::make_unique<Link<double>>();
    link2->setName("link2");

    std::unique_ptr<Link<double>> link3 = std::make_unique<Link<double>>();
    link3->setName("link3");

    // Create some joints
    std::unique_ptr<RevoluteJoint<double>> joint1 = std::make_unique<RevoluteJoint<double>>();
    joint1->setName("joint1");
    joint1->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));

    Translator<double> t(Translator<double>::Generator({ 0.0, 1.0, 0.0 }));
    joint1->setFrame(Motor<double>(t));
    joint1->setLimits(Joint<double>::Limits({ -0.5, 0.5, 1.0, 1.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint2 = std::make_unique<RevoluteJoint<double>>();
    joint2->setName("joint2");
    joint2->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint2->setLimits(Joint<double>::Limits({ -0.8, 0.8, 1.0, 1.0 }));

    t = Translator<double>(Translator<double>::Generator({ 0.0, 1.0, 0.0 }));
    joint2->setFrame(Motor<double>(t));

    // Create the system
    System<double> system;

    REQUIRE(system.getLinks().size() == 0);
    REQUIRE(system.getJoints().size() == 0);
    REQUIRE(!system.hasKinematicChain("unknown"));

    joint1->setParentLink(link1.get());
    link1->addChildJoint(joint1.get());

    joint1->setChildLink(link2.get());
    link2->setParentJoint(joint1.get());

    joint2->setParentLink(link2.get());
    link2->addChildJoint(joint2.get());

    joint2->setChildLink(link3.get());
    link3->setParentJoint(joint2.get());

    system.addLink(std::move(link1));
    system.addLink(std::move(link2));
    system.addLink(std::move(link3));

    system.addJoint(std::move(joint1));
    system.addJoint(std::move(joint2));

    system.finalize();

    REQUIRE(system.hasKinematicChain("joint2"));

    SECTION("Links")
    {
        REQUIRE(system.getBaseLink().getName() == "link1");

        const std::vector<std::unique_ptr<Link<double>>> &links = system.getLinks();
        REQUIRE(links.size() == 3);
        REQUIRE(links[0]->getName() == "link1");
        REQUIRE(links[1]->getName() == "link2");
        REQUIRE(links[2]->getName() == "link3");

        REQUIRE(system.getLink("link1") != nullptr);
        REQUIRE(system.getLink("link2") != nullptr);
        REQUIRE(system.getLink("link3") != nullptr);
    }

    SECTION("Joints")
    {
        const std::vector<std::unique_ptr<Joint<double>>> &joints = system.getJoints();
        REQUIRE(joints.size() == 2);
        REQUIRE(joints[0]->getName() == "joint1");
        REQUIRE(joints[1]->getName() == "joint2");

        REQUIRE(system.getJoint("joint1") != nullptr);
        REQUIRE(system.getJoint("joint2") != nullptr);
    }

    SECTION("Set joint limits")
    {
        typedef Eigen::Matrix<double, 2, 1> Vector;

        system.setJointLimits(Vector({ -1.0, -2.0 }), Vector({ 1.0, 2.0 }));

        SECTION("Get min limits")
        {
            const Eigen::Matrix<double, 2, 1> &limitsMin = system.getJointLimitsMin();

            REQUIRE(limitsMin(0, 0) == Approx(-1.0));
            REQUIRE(limitsMin(1, 0) == Approx(-2.0));
        }

        SECTION("Get max limits")
        {
            const Eigen::Matrix<double, 2, 1> &limitsMax = system.getJointLimitsMax();

            REQUIRE(limitsMax(0, 0) == Approx(1.0));
            REQUIRE(limitsMax(1, 0) == Approx(2.0));
        }

        SECTION("Feasible positions")
        {
            REQUIRE(system.isJointPositionFeasible(Vector({ 0.0, 0.0 })));
            REQUIRE(system.isJointPositionFeasible(Vector({ -0.5, 1.5 })));
            REQUIRE(system.isJointPositionFeasible(Vector({ 0.5, -1.5 })));
        }

        SECTION("Unfeasible position")
        {
            REQUIRE(!system.isJointPositionFeasible(Vector({ 1.5, 0.0 })));
            REQUIRE(!system.isJointPositionFeasible(Vector({ 0.0, 2.5 })));
        }
    }

    SECTION("Default joint limits")
    {
        typedef Eigen::Matrix<double, 2, 1> Vector;

        SECTION("Get min limits")
        {
            const Eigen::Matrix<double, 2, 1> &limitsMin = system.getJointLimitsMin();

            REQUIRE(limitsMin(0, 0) == Approx(-0.5));
            REQUIRE(limitsMin(1, 0) == Approx(-0.8));
        }

        SECTION("Get max limits")
        {
            const Eigen::Matrix<double, 2, 1> &limitsMax = system.getJointLimitsMax();

            REQUIRE(limitsMax(0, 0) == Approx(0.5));
            REQUIRE(limitsMax(1, 0) == Approx(0.8));
        }

        SECTION("Feasible positions")
        {
            REQUIRE(system.isJointPositionFeasible(Vector({ 0.0, 0.0 })));
            REQUIRE(system.isJointPositionFeasible(Vector({ -0.2, 0.5 })));
            REQUIRE(system.isJointPositionFeasible(Vector({ 0.2, -0.5 })));
        }

        SECTION("Unfeasible position")
        {
            REQUIRE(!system.isJointPositionFeasible(Vector({ 1.5, 0.0 })));
            REQUIRE(!system.isJointPositionFeasible(Vector({ 0.0, 2.5 })));
        }
    }

    SECTION("Random configuration")
    {
        auto config = system.getRandomConfiguration();

        REQUIRE(config.rows() == 2);
        REQUIRE(config.cols() == 1);
    }

    SECTION("Kinematic chain")
    {
        auto chain = std::make_unique<KinematicChain<double>>();

        chain->addActuatedJoint(system.getJoint("joint1"));
        chain->addActuatedJoint(system.getJoint("joint2"));

        Motor<double> translator(Translator<double>::Generator({ 0.0, 1.0, 0.0 }));
        chain->addFixedMotor(translator);

        system.addKinematicChain("chain1", std::move(chain));

        REQUIRE(system.hasKinematicChain("chain1"));

        SECTION("Get kinematic chain")
        {
            const KinematicChain<double>* chain2 = system.getKinematicChain("chain1");

            REQUIRE(chain2->getDoF() == 2);

            const std::map<int, Motor<double>> &motors = chain2->getFixedMotors();
            REQUIRE(motors.size() == 1);

            const std::vector<const Joint<double> *> &joints = chain2->getActuatedJoints();
            REQUIRE(joints.size() == 2);
            REQUIRE(joints[0]->getName() == "joint1");
            REQUIRE(joints[1]->getName() == "joint2");
        }

        SECTION("Compute kinematic chain motor")
        {
            Eigen::Vector<double, 2> position({ 0.0, M_PI / 2.0 });
            Motor<double> motor = system.computeKinematicChainMotor("chain1", position);

            REQUIRE(motor.get<blades::scalar>() == Approx(0.7071067812));
            REQUIRE(motor.get<blades::e23>() == Approx(0.0));
            REQUIRE(motor.get<blades::e13>() == Approx(0.0));
            REQUIRE(motor.get<blades::e12>() == Approx(-0.7071067812));
            REQUIRE(motor.get<blades::e1i>() == Approx(-0.3535533906));
            REQUIRE(motor.get<blades::e2i>() == Approx(-1.0606601718));
            REQUIRE(motor.get<blades::e3i>() == Approx(0.0));
            REQUIRE(motor.get<blades::e123i>() == Approx(0.0));
        }

        SECTION("Compute kinematic chain analytic Jacobian")
        {
            Eigen::Vector<double, 2> position({ 0.0, M_PI / 2.0 });

            MultivectorMatrix<double, Motor, 1, 2> jacobian = system.computeKinematicChainAnalyticJacobian("chain1", position);

            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::scalar>() == Approx(-0.353553));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e23>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e13>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e12>() == Approx(-0.353553));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e1i>() == Approx(0.1767766953));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e2i>() == Approx(0.1767766953));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e3i>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e123i>() == Approx(0.0));

            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::scalar>() == Approx(-0.353553));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e23>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e13>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e12>() == Approx(-0.353553));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e1i>() == Approx(-0.1767766953));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e2i>() == Approx(0.5303300859));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e3i>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e123i>() == Approx(0.0));
        }

        SECTION("Compute kinematic chain geometric Jacobian")
        {
            Eigen::Vector<double, 2> position({ 0.0, M_PI / 2.0 });

            MultivectorMatrix<double, MotorGenerator, 1, 2> jacobian = system.computeKinematicChainGeometricJacobian("chain1", position);

            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e23>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e13>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e12>() == Approx(1.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e1i>() == Approx(1.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e2i>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e3i>() == Approx(0.0));

            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e23>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e13>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e12>() == Approx(1.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e1i>() == Approx(2.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e2i>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e3i>() == Approx(0.0));
        }

        SECTION("Compute kinematic chain geometric Jacobian body")
        {
            Eigen::Vector<double, 2> position({ 0.0, M_PI / 2.0 });

            MultivectorMatrix<double, MotorGenerator, 1, 2> jacobian = system.computeKinematicChainGeometricJacobianBody("chain1", position);

            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e23>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e13>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e12>() == Approx(1.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e1i>() == Approx(-1.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e2i>() == Approx(1.0));
            REQUIRE(jacobian.getCoefficient(0, 0).get<blades::e3i>() == Approx(0.0));

            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e23>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e13>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e12>() == Approx(1.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e1i>() == Approx(-1.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e2i>() == Approx(0.0));
            REQUIRE(jacobian.getCoefficient(0, 1).get<blades::e3i>() == Approx(0.0));
        }
    }
}

TEST_CASE("System dynamics", "[System]")
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

    SECTION("Compute inverse dynamics")
    {
        Eigen::Vector<double, 3> position({ 0.0, 0.1, 0.0 });
        Eigen::Vector<double, 3> velocity({ 0.1, 0.4, 0.0 });
        Eigen::Vector<double, 3> acceleration({ -0.2, 0.8, 0.0 });

        Eigen::Vector<double, 3> torques = system.computeInverseDynamics(position, velocity, acceleration);

        REQUIRE(torques(0) == Approx(1.0574));
        REQUIRE(torques(1) == Approx(1.2402));
        REQUIRE(torques(2) == Approx(0.6));
    }

    SECTION("Compute inverse dynamics with shorter kinematic chain")
    {
        std::unique_ptr<KinematicChain<double>> chain = std::make_unique<KinematicChain<double>>();

        chain->addActuatedJoint(system.getJoint("joint1"));
        chain->addActuatedJoint(system.getJoint("joint2"));

        system.addKinematicChain("chain2", std::move(chain));

        Eigen::Vector<double, 2> position({ 0.0, 0.1 });
        Eigen::Vector<double, 2> velocity({ 0.1, 0.4 });
        Eigen::Vector<double, 2> acceleration({ -0.2, 0.8 });

        Eigen::Vector<double, 2> torques = system.computeInverseDynamics(position, velocity, acceleration);

        REQUIRE(torques(0) == Approx(1.0574));
        REQUIRE(torques(1) == Approx(1.2402));
    }

    SECTION("Fail to compute inverse dynamics with incorrect number of DOF")
    {
        Eigen::Vector<double, 2> position({ 0.0, 0.1 });
        Eigen::Vector<double, 2> velocity({ 0.1, 0.4 });
        Eigen::Vector<double, 2> acceleration({ -0.2, 0.8 });

        REQUIRE_THROWS(system.computeInverseDynamics(position, velocity, acceleration));
    }

    SECTION("Compute forward dynamics 2 ")
    {
        Eigen::Vector<double, 3> position({ 0.0, 0.1, 0.0 });
        Eigen::Vector<double, 3> velocity({ 0.1, 0.4, 0.0 });
        Eigen::Vector<double, 3> torques({ 1.0574, 1.2402, 0.6 });

        Eigen::Vector<double, 3> acceleration = system.computeForwardDynamics(position, velocity, torques);

        REQUIRE(acceleration(0) == Approx(-0.2).margin(1e-5));
        REQUIRE(acceleration(1) == Approx(0.8).margin(1e-5));
        REQUIRE(acceleration(2) == Approx(0.0).margin(1e-5));
    }

    SECTION("Compute forward dynamics with shorter kinematic chain")
    {
        std::unique_ptr<KinematicChain<double>> chain = std::make_unique<KinematicChain<double>>();

        chain->addActuatedJoint(system.getJoint("joint1"));
        chain->addActuatedJoint(system.getJoint("joint2"));

        system.addKinematicChain("chain2", std::move(chain));

        Eigen::Vector<double, 2> position({ 0.0, 0.1 });
        Eigen::Vector<double, 2> velocity({ 0.1, 0.4 });
        Eigen::Vector<double, 2> torques({ 1.0574, 1.2402 });

        Eigen::Vector<double, 2> acceleration = system.computeForwardDynamics(position, velocity, torques);

        REQUIRE(acceleration(0) == Approx(-0.2).margin(1e-5));
        REQUIRE(acceleration(1) == Approx(0.8).margin(1e-5));
    }

    SECTION("Fail to compute forward dynamics with incorrect number of DOF")
    {
        Eigen::Vector<double, 2> position({ 0.0, 0.1 });
        Eigen::Vector<double, 2> velocity({ 0.1, 0.4 });
        Eigen::Vector<double, 2> torques({ 1.0574, 1.2402 });

        REQUIRE_THROWS(system.computeForwardDynamics(position, velocity, torques));
    }
}


TEST_CASE("System with fixed joints", "[System]")
{
    // Create the links
    std::unique_ptr<Link<double>> link0 = std::make_unique<Link<double>>();
    link0->setName("link0");
    link0->setMass(1.0);
    link0->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link0->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link1 = std::make_unique<Link<double>>();
    link1->setName("link1");
    link1->setMass(1.0);
    link1->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link1->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link2 = std::make_unique<Link<double>>();
    link2->setName("link2");
    link2->setMass(1.0);
    link2->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link2->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link3 = std::make_unique<Link<double>>();
    link3->setName("link3");
    link3->setMass(1.0);
    link3->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link3->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link4 = std::make_unique<Link<double>>();
    link4->setName("link4");
    link4->setMass(1.0);
    link4->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link4->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link5 = std::make_unique<Link<double>>();
    link5->setName("link5");
    link5->setMass(1.0);
    link5->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link5->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link6 = std::make_unique<Link<double>>();
    link6->setName("link6");
    link6->setMass(1.0);
    link6->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link6->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link7 = std::make_unique<Link<double>>();
    link7->setName("link7");
    link7->setMass(1.0);
    link7->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link7->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    // Create the fixed joints
    std::unique_ptr<FixedJoint<double>> fixed_joint1 = std::make_unique<FixedJoint<double>>();
    fixed_joint1->setName("fixed_joint1");
    fixed_joint1->setFrame(Motor<double>(Translator<double>::Generator({ 0.0, 1.0, 0.0 })));

    std::unique_ptr<FixedJoint<double>> fixed_joint3 = std::make_unique<FixedJoint<double>>();
    fixed_joint3->setName("fixed_joint3");
    fixed_joint3->setFrame(Motor<double>(Translator<double>::Generator({ 0.0, 1.0, 0.0 })));

    std::unique_ptr<FixedJoint<double>> fixed_joint5 = std::make_unique<FixedJoint<double>>();
    fixed_joint5->setName("fixed_joint5");
    fixed_joint5->setFrame(Motor<double>(Translator<double>::Generator({ 1.0, 0.0, 0.0 })));

    std::unique_ptr<FixedJoint<double>> fixed_joint6 = std::make_unique<FixedJoint<double>>();
    fixed_joint6->setName("fixed_joint6");
    fixed_joint6->setFrame(Motor<double>(Translator<double>::Generator({ 1.0, 0.0, 0.0 })));

    // Create the revolute joints
    std::unique_ptr<RevoluteJoint<double>> joint2 = std::make_unique<RevoluteJoint<double>>();
    joint2->setName("joint2");
    joint2->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint2->setFrame(Motor<double>(Translator<double>::Generator({ 0.0, 1.0, 0.0 })));
    joint2->setLimits(Joint<double>::Limits({ -0.5, 0.5, 1.0, 1.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint4 = std::make_unique<RevoluteJoint<double>>();
    joint4->setName("joint4");
    joint4->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint4->setFrame(Motor<double>(Translator<double>::Generator({ 0.0, 1.0, 0.0 })));
    joint4->setLimits(Joint<double>::Limits({ -0.6, 0.6, 1.0, 1.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint7 = std::make_unique<RevoluteJoint<double>>();
    joint7->setName("joint7");
    joint7->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint7->setFrame(Motor<double>(Translator<double>::Generator({ 1.0, 0.0, 0.0 })));
    joint7->setLimits(Joint<double>::Limits({ -0.7, 0.7, 1.0, 1.0 }));

    // Create the system
    System<double> system;

    fixed_joint1->setParentLink(link0.get());
    link0->addChildJoint(fixed_joint1.get());

    fixed_joint1->setChildLink(link1.get());
    link1->setParentJoint(fixed_joint1.get());

    joint2->setParentLink(link1.get());
    link1->addChildJoint(joint2.get());

    joint2->setChildLink(link2.get());
    link2->setParentJoint(joint2.get());

    fixed_joint3->setParentLink(link2.get());
    link2->addChildJoint(fixed_joint3.get());

    fixed_joint3->setChildLink(link3.get());
    link3->setParentJoint(fixed_joint3.get());

    joint4->setParentLink(link3.get());
    link3->addChildJoint(joint4.get());

    joint4->setChildLink(link4.get());
    link4->setParentJoint(joint4.get());

    fixed_joint5->setParentLink(link4.get());
    link4->addChildJoint(fixed_joint5.get());

    fixed_joint5->setChildLink(link5.get());
    link5->setParentJoint(fixed_joint5.get());

    fixed_joint6->setParentLink(link5.get());
    link5->addChildJoint(fixed_joint6.get());

    fixed_joint6->setChildLink(link6.get());
    link6->setParentJoint(fixed_joint6.get());

    joint7->setParentLink(link6.get());
    link6->addChildJoint(joint7.get());

    joint7->setChildLink(link7.get());
    link7->setParentJoint(joint7.get());

    system.addLink(std::move(link0));
    system.addLink(std::move(link1));
    system.addLink(std::move(link2));
    system.addLink(std::move(link3));
    system.addLink(std::move(link4));
    system.addLink(std::move(link5));
    system.addLink(std::move(link6));
    system.addLink(std::move(link7));

    system.addJoint(std::move(fixed_joint1));
    system.addJoint(std::move(joint2));
    system.addJoint(std::move(fixed_joint3));
    system.addJoint(std::move(joint4));
    system.addJoint(std::move(fixed_joint5));
    system.addJoint(std::move(fixed_joint6));
    system.addJoint(std::move(joint7));

    system.finalize();

    REQUIRE(system.hasKinematicChain("joint7"));

    SECTION("Link axes")
    {
        std::vector<std::string> names({"link0", "link1", "link3", "link5", "link6"});
        for (auto name: names) {
            const Link<double>* link = system.getLink(name);
            const typename Motor<double>::Generator axis = link->getAxis();

            REQUIRE(axis.get<blades::e23>() == Approx(0.0));
            REQUIRE(axis.get<blades::e13>() == Approx(0.0));
            REQUIRE(axis.get<blades::e12>() == Approx(0.0));
            REQUIRE(axis.get<blades::e1i>() == Approx(0.0));
            REQUIRE(axis.get<blades::e2i>() == Approx(0.0));
            REQUIRE(axis.get<blades::e3i>() == Approx(0.0));
        }

        names = {"link2", "link4", "link7"};
        for (auto name: names) {
            const Link<double>* link = system.getLink(name);
            const typename Motor<double>::Generator axis = link->getAxis();

            REQUIRE(axis.get<blades::e23>() == Approx(0.0));
            REQUIRE(axis.get<blades::e13>() == Approx(0.0));
            REQUIRE(axis.get<blades::e12>() == Approx(1.0));
            REQUIRE(axis.get<blades::e1i>() == Approx(-0.5));
            REQUIRE(axis.get<blades::e2i>() == Approx(0.0));
            REQUIRE(axis.get<blades::e3i>() == Approx(0.0));
        }
    }

    SECTION("Set joint limits")
    {
        typedef Eigen::Matrix<double, 3, 1> Vector;

        system.setJointLimits(Vector({ -1.0, -2.0, -3.0 }), Vector({ 1.0, 2.0, 3.0 }));

        SECTION("Get min limits")
        {
            const Eigen::Matrix<double, 3, 1> &limitsMin = system.getJointLimitsMin();

            REQUIRE(limitsMin(0, 0) == Approx(-1.0));
            REQUIRE(limitsMin(1, 0) == Approx(-2.0));
            REQUIRE(limitsMin(2, 0) == Approx(-3.0));
        }

        SECTION("Get max limits")
        {
            const Eigen::Matrix<double, 3, 1> &limitsMax = system.getJointLimitsMax();

            REQUIRE(limitsMax(0, 0) == Approx(1.0));
            REQUIRE(limitsMax(1, 0) == Approx(2.0));
            REQUIRE(limitsMax(2, 0) == Approx(3.0));
        }

        SECTION("Feasible positions")
        {
            REQUIRE(system.isJointPositionFeasible(Vector({ 0.0, 0.0, 0.0 })));
            REQUIRE(system.isJointPositionFeasible(Vector({ -0.5, 1.5, 2.5 })));
            REQUIRE(system.isJointPositionFeasible(Vector({ 0.5, -1.5, -2.5 })));
        }

        SECTION("Unfeasible position")
        {
            REQUIRE(!system.isJointPositionFeasible(Vector({ 1.5, 0.0, 0.0 })));
            REQUIRE(!system.isJointPositionFeasible(Vector({ 0.0, 2.5, 0.0 })));
            REQUIRE(!system.isJointPositionFeasible(Vector({ 0.0, 0.0, 3.5 })));
        }
    }

    SECTION("Default joint limits")
    {
        typedef Eigen::Matrix<double, 3, 1> Vector;

        SECTION("Get min limits")
        {
            const Eigen::Matrix<double, 3, 1> &limitsMin = system.getJointLimitsMin();

            REQUIRE(limitsMin(0, 0) == Approx(-0.5));
            REQUIRE(limitsMin(1, 0) == Approx(-0.6));
            REQUIRE(limitsMin(2, 0) == Approx(-0.7));
        }

        SECTION("Get max limits")
        {
            const Eigen::Matrix<double, 3, 1> &limitsMax = system.getJointLimitsMax();

            REQUIRE(limitsMax(0, 0) == Approx(0.5));
            REQUIRE(limitsMax(1, 0) == Approx(0.6));
            REQUIRE(limitsMax(2, 0) == Approx(0.7));
        }

        SECTION("Feasible positions")
        {
            REQUIRE(system.isJointPositionFeasible(Vector({ 0.0, 0.0, 0.0 })));
            REQUIRE(system.isJointPositionFeasible(Vector({ -0.2, 0.5, 0.6 })));
            REQUIRE(system.isJointPositionFeasible(Vector({ 0.2, -0.5, -0.6 })));
        }

        SECTION("Unfeasible position")
        {
            REQUIRE(!system.isJointPositionFeasible(Vector({ 1.5, 0.0, 0.0 })));
            REQUIRE(!system.isJointPositionFeasible(Vector({ 0.0, 2.5, 0.0 })));
            REQUIRE(!system.isJointPositionFeasible(Vector({ 0.0, 0.0, 3.5 })));
        }
    }

    SECTION("Random configuration")
    {
        auto config = system.getRandomConfiguration();

        REQUIRE(config.rows() == 3);
        REQUIRE(config.cols() == 1);
    }

    SECTION("Compute end-effector position 1")
    {
        Eigen::Vector<double, 3> position({ 0.0, 0.0, 0.0 });
        Motor<double> motor = system.computeKinematicChainMotor("joint7", position);

        Point<double> point = motor.apply(Point<double>(0.0, 0.0, 0.0));

        REQUIRE(point.get<blades::e1>() == Approx(3.0));
        REQUIRE(point.get<blades::e2>() == Approx(4.0));
        REQUIRE(point.get<blades::e3>() == Approx(0.0));
        REQUIRE(point.get<blades::ei>() == Approx(12.5));
        REQUIRE(point.get<blades::e0>() == Approx(1.0));
    }

    SECTION("Compute end-effector position 2")
    {
        Eigen::Vector<double, 3> position({ 0.0, M_PI / 2.0, 0.0 });
        Motor<double> motor = system.computeKinematicChainMotor("joint7", position);

        Point<double> point = motor.apply(Point<double>(0.0, 0.0, 0.0));

        REQUIRE(point.get<blades::e1>() == Approx(0.0).margin(1e-6));
        REQUIRE(point.get<blades::e2>() == Approx(7.0));
        REQUIRE(point.get<blades::e3>() == Approx(0.0));
        REQUIRE(point.get<blades::ei>() == Approx(24.5));
        REQUIRE(point.get<blades::e0>() == Approx(1.0));
    }

    SECTION("Get joint torques")
    {
        Eigen::Vector<double, 3> position({ 0.0, 0.1, 0.0 });
        Eigen::Vector<double, 3> velocity({ 0.1, 0.4, 0.0 });
        Eigen::Vector<double, 3> acceleration({ -0.2, 0.8, 0.0 });

        Eigen::Vector<double, 3> torques = system.computeInverseDynamics(position, velocity, acceleration);

        REQUIRE(torques(0) == Approx(4.7830207575));
        REQUIRE(torques(1) == Approx(3.7083281023));
        REQUIRE(torques(2) == Approx(0.1819987506));
    }

    SECTION("Get joint accelerations")
    {
        Eigen::Vector<double, 3> position({ 0.0, 0.1, 0.0 });
        Eigen::Vector<double, 3> velocity({ 0.1, 0.4, 0.0 });
        Eigen::Vector<double, 3> torques({ 4.7830207575, 3.7083281023, 0.1819987506 });

        Eigen::Vector<double, 3> acceleration = system.computeForwardDynamics(position, velocity, torques);

        REQUIRE(acceleration(0) == Approx(-0.2).margin(1e-5));
        REQUIRE(acceleration(1) == Approx(0.8).margin(1e-5));
        REQUIRE(acceleration(2) == Approx(0.0).margin(1e-5));
    }
}


TEST_CASE("System with 7 joints", "[System]")
{
    // Create the links
    std::unique_ptr<Link<double>> link0 = std::make_unique<Link<double>>();
    link0->setName("link0");
    link0->setMass(1.0);
    link0->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link0->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link1 = std::make_unique<Link<double>>();
    link1->setName("link1");
    link1->setMass(1.0);
    link1->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link1->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link2 = std::make_unique<Link<double>>();
    link2->setName("link2");
    link2->setMass(1.0);
    link2->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link2->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link3 = std::make_unique<Link<double>>();
    link3->setName("link3");
    link3->setMass(1.0);
    link3->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link3->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link4 = std::make_unique<Link<double>>();
    link4->setName("link4");
    link4->setMass(1.0);
    link4->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link4->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link5 = std::make_unique<Link<double>>();
    link5->setName("link5");
    link5->setMass(1.0);
    link5->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link5->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link6 = std::make_unique<Link<double>>();
    link6->setName("link6");
    link6->setMass(1.0);
    link6->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link6->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    std::unique_ptr<Link<double>> link7 = std::make_unique<Link<double>>();
    link7->setName("link7");
    link7->setMass(1.0);
    link7->setCenterOfMass(Translator<double>::Generator({ 0.0, 0.5, 0.0 }));
    link7->setInertia(Inertia(1.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.01));

    // Create the revolute joints
    std::unique_ptr<RevoluteJoint<double>> joint1 = std::make_unique<RevoluteJoint<double>>();
    joint1->setName("joint1");
    joint1->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint1->setFrame(Motor<double>(Translator<double>::Generator({ 0.0, 1.0, 0.0 })));

    std::unique_ptr<RevoluteJoint<double>> joint2 = std::make_unique<RevoluteJoint<double>>();
    joint2->setName("joint2");
    joint2->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint2->setFrame(Motor<double>(Translator<double>::Generator({ 0.0, 1.0, 0.0 })));

    std::unique_ptr<RevoluteJoint<double>> joint3 = std::make_unique<RevoluteJoint<double>>();
    joint3->setName("joint3");
    joint3->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint3->setFrame(Motor<double>(Translator<double>::Generator({ 0.0, 1.0, 0.0 })));

    std::unique_ptr<RevoluteJoint<double>> joint4 = std::make_unique<RevoluteJoint<double>>();
    joint4->setName("joint4");
    joint4->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint4->setFrame(Motor<double>(Translator<double>::Generator({ 0.0, 1.0, 0.0 })));

    std::unique_ptr<RevoluteJoint<double>> joint5 = std::make_unique<RevoluteJoint<double>>();
    joint5->setName("joint5");
    joint5->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint5->setFrame(Motor<double>(Translator<double>::Generator({ 1.0, 0.0, 0.0 })));

    std::unique_ptr<RevoluteJoint<double>> joint6 = std::make_unique<RevoluteJoint<double>>();
    joint6->setName("joint6");
    joint6->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint6->setFrame(Motor<double>(Translator<double>::Generator({ 1.0, 0.0, 0.0 })));

    std::unique_ptr<RevoluteJoint<double>> joint7 = std::make_unique<RevoluteJoint<double>>();
    joint7->setName("joint7");
    joint7->setAxis(RevoluteJoint<double>::Axis({ 0.0, 0.0, 1.0 }));
    joint7->setFrame(Motor<double>(Translator<double>::Generator({ 1.0, 0.0, 0.0 })));

    // Create the system
    System<double> system;

    joint1->setParentLink(link0.get());
    link0->addChildJoint(joint1.get());

    joint1->setChildLink(link1.get());
    link1->setParentJoint(joint1.get());

    joint2->setParentLink(link1.get());
    link1->addChildJoint(joint2.get());

    joint2->setChildLink(link2.get());
    link2->setParentJoint(joint2.get());

    joint3->setParentLink(link2.get());
    link2->addChildJoint(joint3.get());

    joint3->setChildLink(link3.get());
    link3->setParentJoint(joint3.get());

    joint4->setParentLink(link3.get());
    link3->addChildJoint(joint4.get());

    joint4->setChildLink(link4.get());
    link4->setParentJoint(joint4.get());

    joint5->setParentLink(link4.get());
    link4->addChildJoint(joint5.get());

    joint5->setChildLink(link5.get());
    link5->setParentJoint(joint5.get());

    joint6->setParentLink(link5.get());
    link5->addChildJoint(joint6.get());

    joint6->setChildLink(link6.get());
    link6->setParentJoint(joint6.get());

    joint7->setParentLink(link6.get());
    link6->addChildJoint(joint7.get());

    joint7->setChildLink(link7.get());
    link7->setParentJoint(joint7.get());

    system.addLink(std::move(link0));
    system.addLink(std::move(link1));
    system.addLink(std::move(link2));
    system.addLink(std::move(link3));
    system.addLink(std::move(link4));
    system.addLink(std::move(link5));
    system.addLink(std::move(link6));
    system.addLink(std::move(link7));

    system.addJoint(std::move(joint1));
    system.addJoint(std::move(joint2));
    system.addJoint(std::move(joint3));
    system.addJoint(std::move(joint4));
    system.addJoint(std::move(joint5));
    system.addJoint(std::move(joint6));
    system.addJoint(std::move(joint7));

    system.finalize();

    REQUIRE(system.hasKinematicChain("joint7"));

    SECTION("Link axes")
    {
        std::vector<std::string> names({"link0"});
        for (auto name: names) {
            const Link<double>* link = system.getLink(name);
            const typename Motor<double>::Generator axis = link->getAxis();

            REQUIRE(axis.get<blades::e23>() == Approx(0.0));
            REQUIRE(axis.get<blades::e13>() == Approx(0.0));
            REQUIRE(axis.get<blades::e12>() == Approx(0.0));
            REQUIRE(axis.get<blades::e1i>() == Approx(0.0));
            REQUIRE(axis.get<blades::e2i>() == Approx(0.0));
            REQUIRE(axis.get<blades::e3i>() == Approx(0.0));
        }

        names = {"link1", "link2", "link3", "link4", "link5", "link6", "link7"};
        for (auto name: names) {
            const Link<double>* link = system.getLink(name);
            const typename Motor<double>::Generator axis = link->getAxis();

            REQUIRE(axis.get<blades::e23>() == Approx(0.0));
            REQUIRE(axis.get<blades::e13>() == Approx(0.0));
            REQUIRE(axis.get<blades::e12>() == Approx(1.0));
            REQUIRE(axis.get<blades::e1i>() == Approx(-0.5));
            REQUIRE(axis.get<blades::e2i>() == Approx(0.0));
            REQUIRE(axis.get<blades::e3i>() == Approx(0.0));
        }
    }

    SECTION("Random configuration")
    {
        auto config = system.getRandomConfiguration();

        REQUIRE(config.rows() == 7);
        REQUIRE(config.cols() == 1);
    }

    SECTION("Compute end-effector position 1")
    {
        Eigen::Vector<double, 7> position({ 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0 });
        Motor<double> motor = system.computeKinematicChainMotor("joint7", position);

        Point<double> point = motor.apply(Point<double>(0.0, 0.0, 0.0));

        REQUIRE(point.get<blades::e1>() == Approx(3.0));
        REQUIRE(point.get<blades::e2>() == Approx(4.0));
        REQUIRE(point.get<blades::e3>() == Approx(0.0));
        REQUIRE(point.get<blades::ei>() == Approx(12.5));
        REQUIRE(point.get<blades::e0>() == Approx(1.0));
    }

    SECTION("Compute end-effector position 2")
    {
        Eigen::Vector<double, 7> position({ 0.0, 0.0, 0.0, M_PI / 2.0, 0.0, 0.0, 0.0 });
        Motor<double> motor = system.computeKinematicChainMotor("joint7", position);

        Point<double> point = motor.apply(Point<double>(0.0, 0.0, 0.0));

        REQUIRE(point.get<blades::e1>() == Approx(0.0).margin(1e-6));
        REQUIRE(point.get<blades::e2>() == Approx(7.0));
        REQUIRE(point.get<blades::e3>() == Approx(0.0));
        REQUIRE(point.get<blades::ei>() == Approx(24.5));
        REQUIRE(point.get<blades::e0>() == Approx(1.0));
    }

    SECTION("Get joint torques")
    {
        Eigen::Vector<double, 7> position({ 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0 });
        Eigen::Vector<double, 7> velocity({ 0.0, 0.1, 0.0, 0.4, 0.0, 0.0, 0.0 });
        Eigen::Vector<double, 7> acceleration({ 0.0, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0 });

        Eigen::Vector<double, 7> torques = system.computeInverseDynamics(position, velocity, acceleration);

        REQUIRE(torques(0) == Approx(11.157));
        REQUIRE(torques(1) == Approx(10.161));
        REQUIRE(torques(2) == Approx(9.11698));
        REQUIRE(torques(3) == Approx(7.87299));
        REQUIRE(torques(4) == Approx(4.86949));
        REQUIRE(torques(5) == Approx(2.15616));
        REQUIRE(torques(6) == Approx(0.332998));
    }

    SECTION("Get joint accelerations")
    {
        Eigen::Vector<double, 7> position({ 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0 });
        Eigen::Vector<double, 7> velocity({ 0.0, 0.1, 0.0, 0.4, 0.0, 0.0, 0.0 });
        Eigen::Vector<double, 7> torques({ 11.157, 10.161, 9.11698, 7.87299, 4.86949, 2.15616,0.332998 });

        Eigen::Vector<double, 7> acceleration = system.computeForwardDynamics(position, velocity, torques);

        REQUIRE(acceleration(0) == Approx(0.0).margin(1e-4));
        REQUIRE(acceleration(1) == Approx(-0.2).margin(1e-2));
        REQUIRE(acceleration(2) == Approx(0.0).margin(1e-2));
        REQUIRE(acceleration(3) == Approx(0.8).margin(1e-3));
        REQUIRE(acceleration(4) == Approx(0.0).margin(1e-4));
        REQUIRE(acceleration(5) == Approx(0.0).margin(1e-4));
        REQUIRE(acceleration(6) == Approx(0.0).margin(1e-4));
    }
}
