#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


void addFinger(const std::string& name, System<double>& system)
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
    auto palm = system.getLink("palm");

    joint1->setParentLink(palm);
    palm->addChildJoint(joint1.get());

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



TEST_CASE("Hand 4 fingers", "[Hand]")
{
    // Create the system
    System<double> system;

    Translator<double> com(Translator<double>::Generator({ 0.0, 0.0, 0.0 }));

    std::unique_ptr<Link<double>> palm = std::make_unique<Link<double>>();
    palm->setName("palm");
    palm->setMass(0.1);
    palm->setCenterOfMass(com);
    palm->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    palm->setAxis(Motor<double>::Generator({ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 }));

    system.addLink(std::move(palm));

    addFinger("finger1", system);
    addFinger("finger2", system);
    addFinger("finger3", system);
    addFinger("finger4", system);

    system.finalize();

    // Create the hand
    Hand<double, 3, 3, 3, 3> hand(
        std::move(system),
        { "finger1_joint3", "finger2_joint3", "finger3_joint3", "finger4_joint3" }
    );

    // Test that some methods at least compile and run, without testing the results
    Eigen::Vector<double, 3> position3 = Eigen::Vector<double, 3>::Zero();
    Eigen::Vector<double, 12> position12 = Eigen::Vector<double, 12>::Zero();
    Motor<double> motor;

    { auto result = hand.getFingerMotor<0>(position3); }
    { auto result = hand.getFingerAnalyticJacobian<0>(position3); }
    { auto result = hand.getFingerGeometricJacobian<0>(position3); }
    { auto result = hand.getFingerGeometricJacobian<0>(position3, motor); }
    { auto result = hand.getFingerMotors(position12); }
    { auto result = hand.getFingerPoints(position12); }
    { auto result = hand.getAnalyticJacobian(position12); }
    { auto result = hand.getGeometricJacobian(position12); }
    { auto result = hand.getGeometricJacobian(position12, motor); }
    { auto result = hand.getMeanMotor(position12); }
    { auto result = hand.getMeanMotorAnalyticJacobian(position12); }
    { auto result = hand.getMeanMotorGeometricJacobian(position12); }
    { auto result = hand.getFingerSphere(position12); }
    { auto result = hand.getFingerSphereJacobian(position12); }
}


TEST_CASE("Hand 3 fingers", "[Hand]")
{
    // Create the system
    System<double> system;

    Translator<double> com(Translator<double>::Generator({ 0.0, 0.0, 0.0 }));

    std::unique_ptr<Link<double>> palm = std::make_unique<Link<double>>();
    palm->setName("palm");
    palm->setMass(0.1);
    palm->setCenterOfMass(com);
    palm->setInertia(Inertia<double>(0.1, Eigen::Matrix<double, 3, 3>::Identity()));
    palm->setAxis(Motor<double>::Generator({ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 }));

    system.addLink(std::move(palm));

    addFinger("finger1", system);
    addFinger("finger2", system);
    addFinger("finger3", system);

    system.finalize();

    // Create the hand
    Hand<double, 3, 3, 3> hand(
        std::move(system),
        { "finger1_joint3", "finger2_joint3", "finger3_joint3" }
    );

    // Test that some methods at least compile and run, without testing the results
    Eigen::Vector<double, 3> position3 = Eigen::Vector<double, 3>::Zero();
    Eigen::Vector<double, 9> position9 = Eigen::Vector<double, 9>::Zero();
    Motor<double> motor;

    { auto result = hand.getFingerMotor<0>(position3); }
    { auto result = hand.getFingerAnalyticJacobian<0>(position3); }
    { auto result = hand.getFingerGeometricJacobian<0>(position3); }
    { auto result = hand.getFingerGeometricJacobian<0>(position3, motor); }
    { auto result = hand.getFingerMotors(position9); }
    { auto result = hand.getFingerPoints(position9); }
    { auto result = hand.getAnalyticJacobian(position9); }
    { auto result = hand.getGeometricJacobian(position9); }
    { auto result = hand.getGeometricJacobian(position9, motor); }
    { auto result = hand.getMeanMotor(position9); }
    { auto result = hand.getMeanMotorAnalyticJacobian(position9); }
    { auto result = hand.getMeanMotorGeometricJacobian(position9); }
    { auto result = hand.getFingerCircle(position9); }
    { auto result = hand.getFingerCircleJacobian(position9); }
}
