#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Default prismatic joint", "[PrismaticJoint]" )
{
    PrismaticJoint<double> joint;

    REQUIRE( joint.getName() == "" );
    REQUIRE( joint.getType() == Joint<double>::Type::PRISMATIC );
    REQUIRE( joint.getParentLink() == nullptr );
    REQUIRE( joint.getChildLink() == nullptr );
    REQUIRE( joint.isActuated() );

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

    SECTION( "has default axis" )
    {
        const Translator<double>::Generator& axis = joint.getAxis();

        REQUIRE( axis.x() == Approx(0.0) );
        REQUIRE( axis.y() == Approx(0.0) );
        REQUIRE( axis.z() == Approx(0.0) );
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

    SECTION( "axis" )
    {
        Translator<double>::Generator generator({ 1.0, 2.0, 3.0 });

        joint.setAxis(generator);

        const Translator<double>::Generator& axis = joint.getAxis();

        REQUIRE( axis.x() == Approx(1.0) );
        REQUIRE( axis.y() == Approx(2.0) );
        REQUIRE( axis.z() == Approx(3.0) );
    }

    SECTION( "get translator" )
    {
        SECTION( "with default axis" )
        {
            Translator<double> translator = joint.getTranslator(10.0);

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
        }

        SECTION( "with custom axis" )
        {
            Translator<double>::Generator generator({ 1.0, 2.0, 3.0 });

            joint.setAxis(generator);

            Translator<double> translator = joint.getTranslator(10.0);

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(-5.0) );
            REQUIRE( translator.get<blades::e2i>() == Approx(-10.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(-15.0) );
        }
    }

    SECTION( "get motor" )
    {
        SECTION( "with default axis" )
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

        SECTION( "with custom axis" )
        {
            Translator<double>::Generator generator({ 1.0, 2.0, 3.0 });
        
            joint.setAxis(generator);

            Motor<double> motor = joint.getMotor(M_PI / 2.0);

            auto rotor = motor.getRotor();

            REQUIRE( rotor.scalar() == Approx(1.0) );
            REQUIRE( rotor.e23() == Approx(0.0) );
            REQUIRE( rotor.e13() == Approx(0.0) );
            REQUIRE( rotor.e12() == Approx(0.0) );
            REQUIRE( rotor.angle() == Approx(0.0) );

            auto translator = motor.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(-0.7853981634) );
            REQUIRE( translator.get<blades::e2i>() == Approx(-1.5707963268) );
            REQUIRE( translator.get<blades::e3i>() == Approx(-2.3561944902) );
        }
    }

    SECTION( "get motor derivative" )
    {
        REQUIRE_THROWS_AS( joint.getMotorDerivative(M_PI / 2.0), std::runtime_error );
    }

    SECTION( "get current axis" )
    {
        SECTION( "with default axis and motor" )
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

        SECTION( "with default axis" )
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

        SECTION( "with custom axis and default motor" )
        {
            Translator<double>::Generator generator({ 1.0, 2.0, 3.0 });
            joint.setAxis(generator);

            Motor<double> motor;

            Motor<double>::Generator axis = joint.getCurrentAxis(motor);

            Rotor<double>::Generator rotorGenerator = axis.getRotorGenerator();
            Translator<double>::Generator translatorGenerator = axis.getTranslatorGenerator();

            REQUIRE( rotorGenerator.e23() == Approx(0.0) );
            REQUIRE( rotorGenerator.e13() == Approx(0.0) );
            REQUIRE( rotorGenerator.e12() == Approx(0.0) );

            REQUIRE( translatorGenerator.x() == Approx(1.0) );
            REQUIRE( translatorGenerator.y() == Approx(2.0) );
            REQUIRE( translatorGenerator.z() == Approx(3.0) );
        }

        SECTION( "with custom axis" )
        {
            Translator<double>::Generator generator({ 1.0, 2.0, 3.0 });
            joint.setAxis(generator);

            Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
            Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
            Motor<double> motor(translator, rotor);

            Motor<double>::Generator axis = joint.getCurrentAxis(motor);

            Rotor<double>::Generator rotorGenerator = axis.getRotorGenerator();
            Translator<double>::Generator translatorGenerator = axis.getTranslatorGenerator();

            REQUIRE( rotorGenerator.e23() == Approx(0.0) );
            REQUIRE( rotorGenerator.e13() == Approx(0.0) );
            REQUIRE( rotorGenerator.e12() == Approx(0.0) );

            REQUIRE( translatorGenerator.x() == Approx(-2.0) );
            REQUIRE( translatorGenerator.y() == Approx(1.0) );
            REQUIRE( translatorGenerator.z() == Approx(3.0) );
        }
    }
}


TEST_CASE( "Prismatic joint with 3 parameters", "[PrismaticJoint]" )
{
    REQUIRE_THROWS_AS( PrismaticJoint<double>({ 1.0, 2.0, 3.0 }), std::runtime_error );
}


TEST_CASE( "Prismatic joint with 6 parameters", "[PrismaticJoint]" )
{
    SECTION( "first configuration" )
    {
        PrismaticJoint<double> joint({ 1.0, 2.0, 3.0, M_PI / 2.0, 0.0, 0.0 }, 1);

        REQUIRE( joint.getName() == "" );
        REQUIRE( joint.getType() == Joint<double>::Type::PRISMATIC );
        REQUIRE( joint.getParentLink() == nullptr );
        REQUIRE( joint.getChildLink() == nullptr );
        REQUIRE( joint.isActuated() );

        SECTION( "frame" )
        {
            const Motor<double>& frame = joint.getFrame();

            auto rotor = frame.getRotor();

            REQUIRE( rotor.scalar() == Approx(0.7071067812) );
            REQUIRE( rotor.e23() == Approx(-0.7071067812) );
            REQUIRE( rotor.e13() == Approx(0.0) );
            REQUIRE( rotor.e12() == Approx(0.0) );
            REQUIRE( rotor.angle() == Approx(M_PI / 2.0) );

            auto translator = frame.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(-0.5) );
            REQUIRE( translator.get<blades::e2i>() == Approx(-1.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(-1.5) );
        }

        SECTION( "axis" )
        {
            const Translator<double>::Generator& axis = joint.getAxis();

            REQUIRE( axis.x() == Approx(1.0) );
            REQUIRE( axis.y() == Approx(0.0) );
            REQUIRE( axis.z() == Approx(0.0) );
        }
    }

    SECTION( "second configuration" )
    {
        PrismaticJoint<double> joint({ 1.0, 2.0, 3.0, 0.0, M_PI / 2.0, 0.0 }, 2);

        REQUIRE( joint.getName() == "" );
        REQUIRE( joint.getType() == Joint<double>::Type::PRISMATIC );
        REQUIRE( joint.getParentLink() == nullptr );
        REQUIRE( joint.getChildLink() == nullptr );
        REQUIRE( joint.isActuated() );

        SECTION( "frame" )
        {
            const Motor<double>& frame = joint.getFrame();

            auto rotor = frame.getRotor();

            REQUIRE( rotor.scalar() == Approx(0.7071067812) );
            REQUIRE( rotor.e23() == Approx(0.0) );
            REQUIRE( rotor.e13() == Approx(-0.7071067812) );
            REQUIRE( rotor.e12() == Approx(0.0) );
            REQUIRE( rotor.angle() == Approx(M_PI / 2.0) );

            auto translator = frame.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(-0.5) );
            REQUIRE( translator.get<blades::e2i>() == Approx(-1.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(-1.5) );
        }

        SECTION( "axis" )
        {
            const Translator<double>::Generator& axis = joint.getAxis();

            REQUIRE( axis.x() == Approx(0.0) );
            REQUIRE( axis.y() == Approx(1.0) );
            REQUIRE( axis.z() == Approx(0.0) );
        }
    }

    SECTION( "third configuration" )
    {
        PrismaticJoint<double> joint({ 1.0, 2.0, 3.0, 0.0, 0.0, M_PI / 2.0 }, -3);

        REQUIRE( joint.getName() == "" );
        REQUIRE( joint.getType() == Joint<double>::Type::PRISMATIC );
        REQUIRE( joint.getParentLink() == nullptr );
        REQUIRE( joint.getChildLink() == nullptr );
        REQUIRE( joint.isActuated() );

        SECTION( "frame" )
        {
            const Motor<double>& frame = joint.getFrame();

            auto rotor = frame.getRotor();

            REQUIRE( rotor.scalar() == Approx(0.7071067812) );
            REQUIRE( rotor.e23() == Approx(0.0) );
            REQUIRE( rotor.e13() == Approx(0.0) );
            REQUIRE( rotor.e12() == Approx(-0.7071067812) );
            REQUIRE( rotor.angle() == Approx(M_PI / 2.0) );

            auto translator = frame.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(-0.5) );
            REQUIRE( translator.get<blades::e2i>() == Approx(-1.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(-1.5) );
        }

        SECTION( "axis" )
        {
            const Translator<double>::Generator& axis = joint.getAxis();

            REQUIRE( axis.x() == Approx(0.0) );
            REQUIRE( axis.y() == Approx(0.0) );
            REQUIRE( axis.z() == Approx(-1.0) );
        }
    }
}
