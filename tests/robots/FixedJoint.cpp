#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Default fixed joint", "[FixedJoint]" )
{
    FixedJoint<double> joint;

    REQUIRE( joint.getName() == "" );
    REQUIRE( joint.getType() == Joint<double>::Type::FIXED );
    REQUIRE( joint.getParentLink() == nullptr );
    REQUIRE( joint.getChildLink() == nullptr );
    REQUIRE( !joint.isActuated() );

    SECTION( "has default frame" )
    {
        const Motor<double>& frame = joint.getFrame();

        auto rotor = frame.getRotor();

        REQUIRE( rotor.scalar() == Approx(1.0) );
        REQUIRE( rotor.e23() == Approx(0.0) );
        REQUIRE( rotor.e13() == Approx(0.0) );
        REQUIRE( rotor.e12() == Approx(0.0) );
        REQUIRE( rotor.angle() == Approx(0.0) );

        auto translator = frame.getTranslator();

        REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
        REQUIRE( translator.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
    }

    SECTION( "frame" )
    {
        Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
        Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
        Motor<double> motor(translator, rotor);

        joint.setFrame(motor);

        const Motor<double>& frame = joint.getFrame();

        auto rotor2 = frame.getRotor();

        REQUIRE( rotor2.scalar() == Approx(rotor.scalar()) );
        REQUIRE( rotor2.e23() == Approx(rotor.e23()) );
        REQUIRE( rotor2.e13() == Approx(rotor.e13()) );
        REQUIRE( rotor2.e12() == Approx(rotor.e12()) );

        auto translator2 = frame.getTranslator();

        REQUIRE( translator2.get<blades::scalar>() == Approx(translator.get<blades::scalar>()) );
        REQUIRE( translator2.get<blades::e1i>() == Approx(translator.get<blades::e1i>()) );
        REQUIRE( translator2.get<blades::e2i>() == Approx(translator.get<blades::e2i>()) );
        REQUIRE( translator2.get<blades::e3i>() == Approx(translator.get<blades::e3i>()) );
    }

    SECTION( "limits" )
    {
        Joint<double>::Limits limits = {1.0, 2.0, 3.0, 4.0};

        joint.setLimits(limits);

        const Joint<double>::Limits& limits2 = joint.getLimits();

        REQUIRE( limits2.position_lower == Approx(1.0) );
        REQUIRE( limits2.position_upper == Approx(2.0) );
        REQUIRE( limits2.velocity == Approx(3.0) );
        REQUIRE( limits2.torque == Approx(4.0) );
    }

    SECTION( "name" )
    {
        joint.setName("joint1");
        REQUIRE( joint.getName() == "joint1" );
    }

    SECTION( "parent link" )
    {
        Link<double> link;

        joint.setParentLink(&link);

        REQUIRE( joint.getParentLink() == &link );
    }

    SECTION( "child link" )
    {
        Link<double> link;

        joint.setChildLink(&link);

        REQUIRE( joint.getChildLink() == &link );
    }

    SECTION( "get motor" )
    {
        Motor<double> motor = joint.getMotor(M_PI / 2.0);

        auto rotor = motor.getRotor();

        REQUIRE( rotor.scalar() == Approx(1.0) );
        REQUIRE( rotor.e23() == Approx(0.0) );
        REQUIRE( rotor.e13() == Approx(0.0) );
        REQUIRE( rotor.e12() == Approx(0.0) );
        REQUIRE( rotor.angle() == Approx(0.0) );

        auto translator = motor.getTranslator();

        REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
        REQUIRE( translator.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
    }

    SECTION( "get motor derivative" )
    {
        SECTION( "with default frame" )
        {
            Motor<double> motor = joint.getMotorDerivative(M_PI / 2.0);

            auto rotor = motor.getRotor();

            REQUIRE( rotor.scalar() == Approx(1.0) );
            REQUIRE( rotor.e23() == Approx(0.0) );
            REQUIRE( rotor.e13() == Approx(0.0) );
            REQUIRE( rotor.e12() == Approx(0.0) );
            REQUIRE( rotor.angle() == Approx(0.0) );

            auto translator = motor.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
        }

        SECTION( "with custom frame" )
        {
            Translator<double> translator2(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
            Rotor<double> rotor2(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
            Motor<double> frame(translator2, rotor2);

            joint.setFrame(frame);

            Motor<double> motor = joint.getMotorDerivative(M_PI / 2.0);

            auto rotor = motor.getRotor();

            REQUIRE( rotor.scalar() == Approx(0.7071067812) );
            REQUIRE( rotor.e23() == Approx(0.0) );
            REQUIRE( rotor.e13() == Approx(0.0) );
            REQUIRE( rotor.e12() == Approx(-0.7071067812) );
            REQUIRE( rotor.angle() == Approx(M_PI / 2.0) );

            auto translator = motor.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(-0.5) );
        }
    }

    SECTION( "get current axis" )
    {
        SECTION( "with default motor" )
        {
            Motor<double> motor;

            Motor<double>::Generator axis = joint.getCurrentAxis(motor);

            Rotor<double>::Generator rotorGenerator = axis.getRotorGenerator();
            Translator<double>::Generator translatorGenerator = axis.getTranslatorGenerator();

            REQUIRE( rotorGenerator.e23() == Approx(0.0) );
            REQUIRE( rotorGenerator.e13() == Approx(0.0) );
            REQUIRE( rotorGenerator.e12() == Approx(0.0) );

            REQUIRE( translatorGenerator.x() == Approx(0.0) );
            REQUIRE( translatorGenerator.y() == Approx(0.0) );
            REQUIRE( translatorGenerator.z() == Approx(0.0) );
        }

        SECTION( "with custom motor" )
        {
            Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
            Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
            Motor<double> motor(translator, rotor);

            Motor<double>::Generator axis = joint.getCurrentAxis(motor);

            Rotor<double>::Generator rotorGenerator = axis.getRotorGenerator();
            Translator<double>::Generator translatorGenerator = axis.getTranslatorGenerator();

            REQUIRE( rotorGenerator.e23() == Approx(0.0) );
            REQUIRE( rotorGenerator.e13() == Approx(0.0) );
            REQUIRE( rotorGenerator.e12() == Approx(0.0) );

            REQUIRE( translatorGenerator.x() == Approx(0.0) );
            REQUIRE( translatorGenerator.y() == Approx(0.0) );
            REQUIRE( translatorGenerator.z() == Approx(0.0) );
        }
    }
}
