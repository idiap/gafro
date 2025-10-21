#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


void addArm(const std::string& name, System<double>& system)
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

    std::unique_ptr<Link<double>> link3 = std::make_unique<Link<double>>();
    link3->setName(name + "_link3");
    link3->setMass(0.1);
    link3->setCenterOfMass(com);
    link3->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    link3->setAxis(Motor<double>::Generator({ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 }));

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

    std::unique_ptr<RevoluteJoint<double>> joint3 = std::make_unique<RevoluteJoint<double>>();
    joint3->setName(name + "_joint3");
    joint3->setAxis(RevoluteJoint<double>::Axis({ 1.0, 0.0, 0.0 }));
    joint3->setLimits(Joint<double>::Limits({ -0.8, 0.8, 1.0, 1.0 }));
    joint3->setFrame(Motor<double>(t));

    // Setup the hierarchy
    auto base_link = system.getLink("base_link");

    joint1->setParentLink(base_link);
    base_link->addChildJoint(joint1.get());

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

    // Add the links and joints to the system
    system.addLink(std::move(link1));
    system.addLink(std::move(link2));
    system.addLink(std::move(link3));

    system.addJoint(std::move(joint1));
    system.addJoint(std::move(joint2));
    system.addJoint(std::move(joint3));
}



TEST_CASE("DualManipulator", "[DualManipulator]")
{
    // Create the system
    System<double> system;

    Translator<double> com(Translator<double>::Generator({ 0.0, 0.0, 0.0 }));

    std::unique_ptr<Link<double>> base_link = std::make_unique<Link<double>>();
    base_link->setName("base_link");
    base_link->setMass(0.1);
    base_link->setCenterOfMass(com);
    base_link->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    base_link->setAxis(Motor<double>::Generator({ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 }));

    system.addLink(std::move(base_link));

    addArm("left", system);
    addArm("right", system);

    system.finalize();

    // Create the hand
    DualManipulator<double, 6> manipulator(std::move(system), "left_joint3", "right_joint3");

    // Test that some methods at least compile and run, without testing the results
    Eigen::Vector<double, 3> position3 = Eigen::Vector<double, 3>::Zero();
    Eigen::Vector<double, 6> position6 = Eigen::Vector<double, 6>::Zero();
    // Eigen::Vector<double, 12> position12 = Eigen::Vector<double, 12>::Zero();
    Motor<double> motor;

    { auto result = manipulator.getFirstBaseMotor(); }
    { auto result = manipulator.getSecondBaseMotor(); }
    { auto result = manipulator.getFirstKinematicChain(); }
    { auto result = manipulator.getSecondKinematicChain(); }
    { auto result = manipulator.getFirstEEMotor(position3); }
    { auto result = manipulator.getSecondEEMotor(position3); }
    { auto result = manipulator.getAbsoluteMotor(position3, position3); }
    { auto result = manipulator.getAbsoluteMotor(position6); }
    { auto result = manipulator.getRelativeMotor(position3, position3); }
    { auto result = manipulator.getRelativeMotor(position6); }
    { auto result = manipulator.getEEPointPair(position3, position3); }
    { auto result = manipulator.getRelativeAnalyticJacobian(position6); }
    { auto result = manipulator.getRelativeAnalyticJacobian(position3, position3); }
    { auto result = manipulator.getRelativeGeometricJacobian(position6, motor); }
    { auto result = manipulator.getRelativeGeometricJacobian(position3, position3, motor); }
    { auto result = manipulator.getRelativeGeometricJacobianTimeDerivative(position6, position6, motor); }
    { auto result = manipulator.getRelativeGeometricJacobianTimeDerivative(position3, position3, position3, position3, motor); }
    { auto result = manipulator.getAbsoluteAnalyticJacobian(position6); }
    { auto result = manipulator.getAbsoluteAnalyticJacobian(position3, position3); }
    { auto result = manipulator.getAbsoluteGeometricJacobian(position6, motor); }
    { auto result = manipulator.getAbsoluteGeometricJacobian(position3, position3, motor); }
    { auto result = manipulator.getAbsoluteGeometricJacobianTimeDerivative(position6, position6, motor); }
    { auto result = manipulator.getAbsoluteGeometricJacobianTimeDerivative(position3, position3, position3, position3, motor); }
    { auto result = manipulator.getPointPairJacobian(position3, position3); }
    { auto result = manipulator.getRelativeVelocityManipulability(position6); }
    { auto result = manipulator.getRelativeVelocityManipulability(position3, position3); }
    { auto result = manipulator.getRelativeForceManipulability(position6); }
    { auto result = manipulator.getRelativeForceManipulability(position3, position3); }
    { auto result = manipulator.getRelativeDynamicManipulability(position6); }
    { auto result = manipulator.getRelativeDynamicManipulability(position3, position3); }
    { auto result = manipulator.getAbsoluteVelocityManipulability(position6); }
    { auto result = manipulator.getAbsoluteVelocityManipulability(position3, position3); }
    { auto result = manipulator.getAbsoluteForceManipulability(position6); }
    { auto result = manipulator.getAbsoluteForceManipulability(position3, position3); }
    { auto result = manipulator.getAbsoluteDynamicManipulability(position6); }
    { auto result = manipulator.getAbsoluteDynamicManipulability(position3, position3); }
    { auto result = manipulator.getJointTorques(position6, position6, position6, 10.0); }
}
