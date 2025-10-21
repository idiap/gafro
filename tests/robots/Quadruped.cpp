#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


void addLeg(const std::string& name, System<double>& system)
{
    // Create the links
    Translator<double> com(Translator<double>::Generator({ 0.0, 0.0, 0.0 }));

    std::unique_ptr<Link<double>> link1 = std::make_unique<Link<double>>();
    link1->setName(name + "_link1");
    link1->setMass(0.1);
    link1->setCenterOfMass(com);
    link1->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    link1->setAxis(Motor<double>::Generator({ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 }));

    std::unique_ptr<Link<double>> link2 = std::make_unique<Link<double>>();
    link2->setName(name + "_link2");
    link2->setMass(0.1);
    link2->setCenterOfMass(com);
    link2->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    link2->setAxis(Motor<double>::Generator({ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 }));

    // Create the joints
    Translator<double> t(Translator<double>::Generator({ 0.0, 1.0, 0.0 }));

    std::unique_ptr<RevoluteJoint<double>> joint1 = std::make_unique<RevoluteJoint<double>>();
    joint1->setName(name + "_joint1");
    joint1->setAxis(RevoluteJoint<double>::Axis({ 1.0, 0.0, 0.0 }));
    joint1->setLimits(Joint<double>::Limits({ -0.5, 0.5, 1.0, 1.0 }));
    joint1->setFrame(Motor<double>(t));

    std::unique_ptr<RevoluteJoint<double>> joint2 = std::make_unique<RevoluteJoint<double>>();
    joint2->setName(name + "_joint2");
    joint2->setAxis(RevoluteJoint<double>::Axis({ 1.0, 0.0, 0.0 }));
    joint2->setLimits(Joint<double>::Limits({ -0.8, 0.8, 1.0, 1.0 }));
    joint2->setFrame(Motor<double>(t));

    // Setup the hierarchy
    auto body = system.getLink("body");

    joint1->setParentLink(body);
    body->addChildJoint(joint1.get());

    joint1->setChildLink(link1.get());
    link1->setParentJoint(joint1.get());

    joint2->setParentLink(link1.get());
    link1->addChildJoint(joint2.get());

    joint2->setChildLink(link2.get());
    link2->setParentJoint(joint2.get());

    // Add the links and joints to the system
    system.addLink(std::move(link1));
    system.addLink(std::move(link2));

    system.addJoint(std::move(joint1));
    system.addJoint(std::move(joint2));
}



TEST_CASE("Quadruped", "[Quadruped]")
{
    // Create the system
    System<double> system;

    Translator<double> com(Translator<double>::Generator({ 0.0, 0.0, 0.0 }));

    std::unique_ptr<Link<double>> body = std::make_unique<Link<double>>();
    body->setName("body");
    body->setMass(0.1);
    body->setCenterOfMass(com);
    body->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    body->setAxis(Motor<double>::Generator({ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 }));

    system.addLink(std::move(body));

    addLeg("leg1", system);
    addLeg("leg2", system);
    addLeg("leg3", system);
    addLeg("leg4", system);

    system.finalize();

    // Create the manipulator
    Quadruped<double, 2> quadruped(
        std::move(system),
        { "leg1_joint2", "leg2_joint2", "leg3_joint2", "leg4_joint2" }
    );

    // Test that some methods at least compile and run, without testing the results
    Eigen::Vector<double, 2> position2 = Eigen::Vector<double, 2>::Zero();
    Eigen::Vector<double, 8> position8 = Eigen::Vector<double, 8>::Zero();
    Motor<double> motor;

    { auto result = quadruped.getFootMotor(0, position2); }
    { auto result = quadruped.getFootMotors(position8); }
    { auto result = quadruped.getFootPoints(position8); }
    { auto result = quadruped.getFootSphere(position8); }
//    { auto result = quadruped.getFootSphereJacobian(position8); }
    { auto result = quadruped.getFootAnalyticJacobian(0, position2); }
    { auto result = quadruped.getFootGeometricJacobian(0, position2); }
    { auto result = quadruped.getFootGeometricJacobian(0, position2, motor); }
    { auto result = quadruped.getAnalyticJacobian(position8); }
    { auto result = quadruped.getGeometricJacobian(position8); }
    { auto result = quadruped.getGeometricJacobian(position8, motor); }
    { auto result = quadruped.getMeanMotor(position8); }
    { auto result = quadruped.getMeanMotorAnalyticJacobian(position8); }
    { auto result = quadruped.getMeanMotorGeometricJacobian(position8); }
}
