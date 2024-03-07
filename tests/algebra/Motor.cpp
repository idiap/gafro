#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Default motor", "[Motor]" )
{
    Motor<double> motor;

    SECTION( "rotor" )
    {
        auto rotor = motor.getRotor();

        REQUIRE( rotor.scalar() == Approx(1.0) );
        REQUIRE( rotor.e23() == Approx(0.0) );
        REQUIRE( rotor.e13() == Approx(0.0) );
        REQUIRE( rotor.e12() == Approx(0.0) );
    }

    SECTION( "translator" )
    {
        auto translator = motor.getTranslator();

        REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
        REQUIRE( translator.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
    }

    SECTION( "log" )
    {
        auto log = motor.log();

        REQUIRE( log.get<blades::e23>() == Approx(0.0) );
        REQUIRE( log.get<blades::e13>() == Approx(0.0) );
        REQUIRE( log.get<blades::e12>() == Approx(0.0) );
        REQUIRE( log.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( log.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( log.get<blades::e3i>() == Approx(0.0) );
    }

    SECTION( "logJacobian" )
    {
        auto jacobian = motor.logJacobian();

        REQUIRE( jacobian.coeff(0, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(1, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(2, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(3, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 4) == Approx(-2.0) );
        REQUIRE( jacobian.coeff(3, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(4, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 5) == Approx(-2.0) );
        REQUIRE( jacobian.coeff(4, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(5, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 6) == Approx(-2.0) );
        REQUIRE( jacobian.coeff(5, 7) == Approx(0.0) );
    }

    SECTION( "apply to point" )
    {
        SECTION( "origin" )
        {
            Point<double> point;

            Point<double> result = motor.apply(point);

            REQUIRE( result.get<blades::e0>() == Approx(point.get<blades::e0>()) );
            REQUIRE( result.get<blades::e1>() == Approx(point.get<blades::e1>()) );
            REQUIRE( result.get<blades::e2>() == Approx(point.get<blades::e2>()) );
            REQUIRE( result.get<blades::e3>() == Approx(point.get<blades::e3>()) );
            REQUIRE( result.get<blades::ei>() == Approx(point.get<blades::ei>()) );
        }

        SECTION( "other" )
        {
            Point<double> point(1.0, 2.0, 3.0);

            Point<double> result = motor.apply(point);

            REQUIRE( result.get<blades::e0>() == Approx(point.get<blades::e0>()) );
            REQUIRE( result.get<blades::e1>() == Approx(point.get<blades::e1>()) );
            REQUIRE( result.get<blades::e2>() == Approx(point.get<blades::e2>()) );
            REQUIRE( result.get<blades::e3>() == Approx(point.get<blades::e3>()) );
            REQUIRE( result.get<blades::ei>() == Approx(point.get<blades::ei>()) );
        }
    }

    SECTION( "apply to line" )
    {
        Point<double> p1;
        Point<double> p2(1.0, 0.0, 0.0);
        Line<double> line(p1, p2);

        Line<double> result = motor.apply(line);

        REQUIRE( result.get<blades::e12i>() == Approx(line.get<blades::e12i>()) );
        REQUIRE( result.get<blades::e13i>() == Approx(line.get<blades::e13i>()) );
        REQUIRE( result.get<blades::e23i>() == Approx(line.get<blades::e23i>()) );
        REQUIRE( result.get<blades::e01i>() == Approx(line.get<blades::e01i>()) );
        REQUIRE( result.get<blades::e02i>() == Approx(line.get<blades::e02i>()) );
        REQUIRE( result.get<blades::e03i>() == Approx(line.get<blades::e03i>()) );
    }

    SECTION( "apply to plane" )
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 0.0, 0.0);
        Point<double> p3(0.0, 1.0, 0.0);
        Plane<double> plane(p1, p2, p3);

        Plane<double> result = motor.apply(plane);

        REQUIRE( result.get<blades::e123i>() == Approx(plane.get<blades::e123i>()) );
        REQUIRE( result.get<blades::e012i>() == Approx(plane.get<blades::e012i>()) );
        REQUIRE( result.get<blades::e023i>() == Approx(plane.get<blades::e023i>()) );
        REQUIRE( result.get<blades::e013i>() == Approx(plane.get<blades::e013i>()) );
    }

    SECTION( "apply to sphere" )
    {
        Point<double> center(1.0, 0.0, 0.0);
        Sphere<double> sphere(center, 0.5);

        Sphere<double> result = motor.apply(sphere);

        Point<double> resultCenter = result.getCenter();

        REQUIRE( resultCenter.get<blades::e0>() == Approx(center.get<blades::e0>()) );
        REQUIRE( resultCenter.get<blades::e1>() == Approx(center.get<blades::e1>()) );
        REQUIRE( resultCenter.get<blades::e2>() == Approx(center.get<blades::e2>()) );
        REQUIRE( resultCenter.get<blades::e3>() == Approx(center.get<blades::e3>()) );
        REQUIRE( resultCenter.get<blades::ei>() == Approx(center.get<blades::ei>()) );

        REQUIRE( result.getRadius() == Approx(sphere.getRadius()) );
    }


    SECTION( "to point pair" )
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 0.0, 0.0);

        PointPair<double> pair(p1, p2);

        PointPair<double> result = motor.apply(pair);

        Point<double> resultPoint1 = result.getPoint1();

        REQUIRE( resultPoint1.get<blades::e1>() == Approx(p1.get<blades::e1>()) );
        REQUIRE( resultPoint1.get<blades::e2>() == Approx(p1.get<blades::e2>()) );
        REQUIRE( resultPoint1.get<blades::e3>() == Approx(p1.get<blades::e3>()) );
        REQUIRE( resultPoint1.get<blades::ei>() == Approx(p1.get<blades::ei>()) );
        REQUIRE( resultPoint1.get<blades::e0>() == Approx(p1.get<blades::e0>()) );

        Point<double> resultPoint2 = result.getPoint2();

        REQUIRE( resultPoint2.get<blades::e1>() == Approx(p2.get<blades::e1>()) );
        REQUIRE( resultPoint2.get<blades::e2>() == Approx(p2.get<blades::e2>()) );
        REQUIRE( resultPoint2.get<blades::e3>() == Approx(p2.get<blades::e3>()) );
        REQUIRE( resultPoint2.get<blades::ei>() == Approx(p2.get<blades::ei>()) );
        REQUIRE( resultPoint2.get<blades::e0>() == Approx(p2.get<blades::e0>()) );
    }

    SECTION( "to wrench" )
    {
        Wrench<double> wrench({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

        Wrench<double> result = motor.apply(wrench);

        REQUIRE( wrench.get<blades::e23>() == Approx(1.0) );
        REQUIRE( wrench.get<blades::e13>() == Approx(2.0) );
        REQUIRE( wrench.get<blades::e12>() == Approx(3.0) );
        REQUIRE( wrench.get<blades::e01>() == Approx(4.0) );
        REQUIRE( wrench.get<blades::e02>() == Approx(5.0) );
        REQUIRE( wrench.get<blades::e03>() == Approx(6.0) );
    }

    SECTION( "to twist" )
    {
        Twist<double> twist({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

        Twist<double> result = motor.apply(twist);

        REQUIRE( result.get<blades::e23>() == Approx(1.0) );
        REQUIRE( result.get<blades::e13>() == Approx(2.0) );
        REQUIRE( result.get<blades::e12>() == Approx(3.0) );
        REQUIRE( result.get<blades::e1i>() == Approx(4.0) );
        REQUIRE( result.get<blades::e2i>() == Approx(5.0) );
        REQUIRE( result.get<blades::e3i>() == Approx(6.0) );
    }
}


TEST_CASE( "Motor with translation", "[Motor]" )
{
    Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
    Motor<double> motor(translator);

    SECTION( "rotor" )
    {
        auto rotor = motor.getRotor();

        REQUIRE( rotor.scalar() == Approx(1.0) );
        REQUIRE( rotor.e23() == Approx(0.0) );
        REQUIRE( rotor.e13() == Approx(0.0) );
        REQUIRE( rotor.e12() == Approx(0.0) );
    }

    SECTION( "translator" )
    {
        auto translator2 = motor.getTranslator();

        REQUIRE( translator2.get<blades::scalar>() == Approx(translator.get<blades::scalar>()) );
        REQUIRE( translator2.get<blades::e1i>() == Approx(translator.get<blades::e1i>()) );
        REQUIRE( translator2.get<blades::e2i>() == Approx(translator.get<blades::e2i>()) );
        REQUIRE( translator2.get<blades::e3i>() == Approx(translator.get<blades::e3i>()) );
    }

    SECTION( "log" )
    {
        auto log = motor.log();

        REQUIRE( log.get<blades::e23>() == Approx(0.0) );
        REQUIRE( log.get<blades::e13>() == Approx(0.0) );
        REQUIRE( log.get<blades::e12>() == Approx(0.0) );
        REQUIRE( log.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( log.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( log.get<blades::e3i>() == Approx(1.0) );
    }

    SECTION( "logJacobian" )
    {
        auto jacobian = motor.logJacobian();

        REQUIRE( jacobian.coeff(0, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(1, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(2, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(3, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 2) == Approx(1.0) );
        REQUIRE( jacobian.coeff(3, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 4) == Approx(-2.0) );
        REQUIRE( jacobian.coeff(3, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(4, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 1) == Approx(1.0) );
        REQUIRE( jacobian.coeff(4, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 5) == Approx(-2.0) );
        REQUIRE( jacobian.coeff(4, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(5, 0) == Approx(1.0) );
        REQUIRE( jacobian.coeff(5, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 6) == Approx(-2.0) );
        REQUIRE( jacobian.coeff(5, 7) == Approx(0.0) );
    }

    SECTION( "apply to point" )
    {
        SECTION( "origin" )
        {
            Point<double> point;

            Point<double> result = motor.apply(point);

            REQUIRE( result.get<blades::e0>() == Approx(1.0) );
            REQUIRE( result.get<blades::e1>() == Approx(0.0) );
            REQUIRE( result.get<blades::e2>() == Approx(0.0) );
            REQUIRE( result.get<blades::e3>() == Approx(1.0) );
            REQUIRE( result.get<blades::ei>() == Approx(0.5) );
        }

        SECTION( "other" )
        {
            Point<double> point(1.0, 2.0, 3.0);

            Point<double> result = motor.apply(point);

            REQUIRE( result.get<blades::e0>() == Approx(1.0) );
            REQUIRE( result.get<blades::e1>() == Approx(1.0) );
            REQUIRE( result.get<blades::e2>() == Approx(2.0) );
            REQUIRE( result.get<blades::e3>() == Approx(4.0) );
            REQUIRE( result.get<blades::ei>() == Approx(10.5) );
        }
    }

    SECTION( "apply to line" )
    {
        Point<double> p1;
        Point<double> p2(1.0, 0.0, 0.0);
        Line<double> line(p1, p2);

        Line<double> result = motor.apply(line);

        REQUIRE( result.get<blades::e12i>() == Approx(line.get<blades::e12i>()) );
        REQUIRE( result.get<blades::e13i>() == Approx(-1.0) );
        REQUIRE( result.get<blades::e23i>() == Approx(line.get<blades::e23i>()) );
        REQUIRE( result.get<blades::e01i>() == Approx(line.get<blades::e01i>()) );
        REQUIRE( result.get<blades::e02i>() == Approx(line.get<blades::e02i>()) );
        REQUIRE( result.get<blades::e03i>() == Approx(line.get<blades::e03i>()) );
    }

    SECTION( "apply to plane" )
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 0.0, 0.0);
        Point<double> p3(0.0, 1.0, 0.0);
        Plane<double> plane(p1, p2, p3);

        Plane<double> result = motor.apply(plane);

        REQUIRE( result.get<blades::e123i>() == Approx(1.0) );
        REQUIRE( result.get<blades::e012i>() == Approx(plane.get<blades::e012i>()) );
        REQUIRE( result.get<blades::e023i>() == Approx(plane.get<blades::e023i>()) );
        REQUIRE( result.get<blades::e013i>() == Approx(plane.get<blades::e013i>()) );
    }

    SECTION( "apply to sphere" )
    {
        Point<double> center(1.0, 0.0, 0.0);
        Sphere<double> sphere(center, 0.5);

        Sphere<double> result = motor.apply(sphere);

        Point<double> resultCenter = result.getCenter();

        REQUIRE( resultCenter.get<blades::e0>() == Approx(center.get<blades::e0>()) );
        REQUIRE( resultCenter.get<blades::e1>() == Approx(center.get<blades::e1>()) );
        REQUIRE( resultCenter.get<blades::e2>() == Approx(center.get<blades::e2>()) );
        REQUIRE( resultCenter.get<blades::e3>() == Approx(1.0) );
        REQUIRE( resultCenter.get<blades::ei>() == Approx(1.0) );

        REQUIRE( result.getRadius() == Approx(sphere.getRadius()) );
    }

    SECTION( "to point pair" )
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 0.0, 0.0);

        PointPair<double> pair(p1, p2);

        PointPair<double> result = motor.apply(pair);

        Point<double> resultPoint1 = result.getPoint1();

        REQUIRE( resultPoint1.get<blades::e1>() == Approx(0.0) );
        REQUIRE( resultPoint1.get<blades::e2>() == Approx(0.0) );
        REQUIRE( resultPoint1.get<blades::e3>() == Approx(1.0) );
        REQUIRE( resultPoint1.get<blades::ei>() == Approx(0.5) );
        REQUIRE( resultPoint1.get<blades::e0>() == Approx(1.0) );

        Point<double> resultPoint2 = result.getPoint2();

        REQUIRE( resultPoint2.get<blades::e1>() == Approx(1.0) );
        REQUIRE( resultPoint2.get<blades::e2>() == Approx(0.0) );
        REQUIRE( resultPoint2.get<blades::e3>() == Approx(1.0) );
        REQUIRE( resultPoint2.get<blades::ei>() == Approx(1.0) );
        REQUIRE( resultPoint2.get<blades::e0>() == Approx(1.0) );
    }

    SECTION( "to wrench" )
    {
        Wrench<double> wrench({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

        Wrench<double> result = motor.apply(wrench);

        REQUIRE( result.get<blades::e23>() == Approx(-4.0) );
        REQUIRE( result.get<blades::e13>() == Approx(-2.0) );
        REQUIRE( result.get<blades::e12>() == Approx(3.0) );
        REQUIRE( result.get<blades::e01>() == Approx(4.0) );
        REQUIRE( result.get<blades::e02>() == Approx(5.0) );
        REQUIRE( result.get<blades::e03>() == Approx(6.0) );
    }

    SECTION( "to twist" )
    {
        Twist<double> twist({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

        Twist<double> result = motor.apply(twist);

        REQUIRE( result.get<blades::e23>() == Approx(1.0) );
        REQUIRE( result.get<blades::e13>() == Approx(2.0) );
        REQUIRE( result.get<blades::e12>() == Approx(3.0) );
        REQUIRE( result.get<blades::e1i>() == Approx(6.0) );
        REQUIRE( result.get<blades::e2i>() == Approx(6.0) );
        REQUIRE( result.get<blades::e3i>() == Approx(6.0) );
    }
}


TEST_CASE( "Motor with rotation", "[Motor]" )
{
    Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
    Motor<double> motor(rotor);

    SECTION( "rotor" )
    {
        auto rotor2 = motor.getRotor();

        REQUIRE( rotor2.scalar() == Approx(rotor.scalar()) );
        REQUIRE( rotor2.e23() == Approx(rotor.e23()) );
        REQUIRE( rotor2.e13() == Approx(rotor.e13()) );
        REQUIRE( rotor2.e12() == Approx(rotor.e12()) );
    }

    SECTION( "translator" )
    {
        auto translator = motor.getTranslator();

        REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
        REQUIRE( translator.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
    }

    SECTION( "log" )
    {
        auto log = motor.log();

        REQUIRE( log.get<blades::e23>() == Approx(0.0) );
        REQUIRE( log.get<blades::e13>() == Approx(0.0) );
        REQUIRE( log.get<blades::e12>() == Approx(1.5707963266) );
        REQUIRE( log.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( log.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( log.get<blades::e3i>() == Approx(0.0) );
    }

    SECTION( "logJacobian" )
    {
        auto jacobian = motor.logJacobian();

        REQUIRE( jacobian.coeff(0, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 1) == Approx(-2.22144) );
        REQUIRE( jacobian.coeff(0, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(1, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 2) == Approx(-2.22144) );
        REQUIRE( jacobian.coeff(1, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(2, 0) == Approx(-0.606986) );
        REQUIRE( jacobian.coeff(2, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 3) == Approx(-2.22144) );
        REQUIRE( jacobian.coeff(2, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(3, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 4) == Approx(-1.41421) );
        REQUIRE( jacobian.coeff(3, 5) == Approx(1.41421) );
        REQUIRE( jacobian.coeff(3, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(4, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 4) == Approx(-1.41421) );
        REQUIRE( jacobian.coeff(4, 5) == Approx(-1.41421) );
        REQUIRE( jacobian.coeff(4, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(5, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 6) == Approx(-1.41421) );
        REQUIRE( jacobian.coeff(5, 7) == Approx(1.41421) );
    }

    SECTION( "apply to point" )
    {
        SECTION( "origin" )
        {
            Point<double> point;

            Point<double> result = motor.apply(point);

            REQUIRE( result.get<blades::e0>() == Approx(point.get<blades::e0>()) );
            REQUIRE( result.get<blades::e1>() == Approx(point.get<blades::e1>()) );
            REQUIRE( result.get<blades::e2>() == Approx(point.get<blades::e2>()) );
            REQUIRE( result.get<blades::e3>() == Approx(point.get<blades::e3>()) );
            REQUIRE( result.get<blades::ei>() == Approx(point.get<blades::ei>()) );
        }

        SECTION( "other" )
        {
            Point<double> point(1.0, 2.0, 3.0);

            Point<double> result = motor.apply(point);

            REQUIRE( result.get<blades::e0>() == Approx(1.0) );
            REQUIRE( result.get<blades::e1>() == Approx(-2.0) );
            REQUIRE( result.get<blades::e2>() == Approx(1.0) );
            REQUIRE( result.get<blades::e3>() == Approx(3.0) );
            REQUIRE( result.get<blades::ei>() == Approx(7.0) );
        }
    }

    SECTION( "apply to line" )
    {
        Point<double> p1;
        Point<double> p2(1.0, 0.0, 0.0);
        Line<double> line(p1, p2);

        Line<double> result = motor.apply(line);

        REQUIRE( result.get<blades::e12i>() == Approx(line.get<blades::e12i>()) );
        REQUIRE( result.get<blades::e13i>() == Approx(line.get<blades::e13i>()) );
        REQUIRE( result.get<blades::e23i>() == Approx(line.get<blades::e23i>()) );
        REQUIRE( result.get<blades::e01i>() == Approx(0.0).margin(1e-6) );
        REQUIRE( result.get<blades::e02i>() == Approx(1.0) );
        REQUIRE( result.get<blades::e03i>() == Approx(line.get<blades::e03i>()) );
    }

    SECTION( "apply to plane" )
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 0.0, 0.0);
        Point<double> p3(0.0, 1.0, 0.0);
        Plane<double> plane(p1, p2, p3);

        Plane<double> result = motor.apply(plane);

        REQUIRE( result.get<blades::e123i>() == Approx(plane.get<blades::e123i>()) );
        REQUIRE( result.get<blades::e012i>() == Approx(plane.get<blades::e012i>()) );
        REQUIRE( result.get<blades::e023i>() == Approx(plane.get<blades::e023i>()) );
        REQUIRE( result.get<blades::e013i>() == Approx(plane.get<blades::e013i>()) );
    }

    SECTION( "apply to sphere" )
    {
        Point<double> center(1.0, 0.0, 0.0);
        Sphere<double> sphere(center, 0.5);

        Sphere<double> result = motor.apply(sphere);

        Point<double> resultCenter = result.getCenter();

        REQUIRE( resultCenter.get<blades::e0>() == Approx(center.get<blades::e0>()) );
        REQUIRE( resultCenter.get<blades::e1>() == Approx(0.0).margin(1e-6) );
        REQUIRE( resultCenter.get<blades::e2>() == Approx(1.0) );
        REQUIRE( resultCenter.get<blades::e3>() == Approx(center.get<blades::e3>()) );
        REQUIRE( resultCenter.get<blades::ei>() == Approx(center.get<blades::ei>()) );

        REQUIRE( result.getRadius() == Approx(sphere.getRadius()) );
    }

    SECTION( "to point pair" )
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 0.0, 0.0);

        PointPair<double> pair(p1, p2);

        PointPair<double> result = motor.apply(pair);

        Point<double> resultPoint1 = result.getPoint1();

        REQUIRE( resultPoint1.get<blades::e1>() == Approx(0.0) );
        REQUIRE( resultPoint1.get<blades::e2>() == Approx(0.0) );
        REQUIRE( resultPoint1.get<blades::e3>() == Approx(0.0) );
        REQUIRE( resultPoint1.get<blades::ei>() == Approx(0.0) );
        REQUIRE( resultPoint1.get<blades::e0>() == Approx(1.0) );

        Point<double> resultPoint2 = result.getPoint2();

        REQUIRE( resultPoint2.get<blades::e1>() == Approx(0.0).margin(1e-6) );
        REQUIRE( resultPoint2.get<blades::e2>() == Approx(1.0) );
        REQUIRE( resultPoint2.get<blades::e3>() == Approx(0.0) );
        REQUIRE( resultPoint2.get<blades::ei>() == Approx(0.5) );
        REQUIRE( resultPoint2.get<blades::e0>() == Approx(1.0) );
    }

    SECTION( "to wrench" )
    {
        Wrench<double> wrench({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

        Wrench<double> result = motor.apply(wrench);

        REQUIRE( result.get<blades::e23>() == Approx(2.0) );
        REQUIRE( result.get<blades::e13>() == Approx(-1.0) );
        REQUIRE( result.get<blades::e12>() == Approx(3.0) );
        REQUIRE( result.get<blades::e01>() == Approx(-5.0) );
        REQUIRE( result.get<blades::e02>() == Approx(4.0) );
        REQUIRE( result.get<blades::e03>() == Approx(6.0) );
    }

    SECTION( "to twist" )
    {
        Twist<double> twist({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

        Twist<double> result = motor.apply(twist);

        REQUIRE( result.get<blades::e23>() == Approx(2.0) );
        REQUIRE( result.get<blades::e13>() == Approx(-1.0) );
        REQUIRE( result.get<blades::e12>() == Approx(3.0) );
        REQUIRE( result.get<blades::e1i>() == Approx(-5.0) );
        REQUIRE( result.get<blades::e2i>() == Approx(4.0) );
        REQUIRE( result.get<blades::e3i>() == Approx(6.0) );
    }
}


TEST_CASE( "Motor with translation & rotation", "[Motor]" )
{
    Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
    Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
    Motor<double> motor(translator, rotor);

    SECTION( "rotor" )
    {
        auto rotor2 = motor.getRotor();

        REQUIRE( rotor2.scalar() == Approx(rotor.scalar()) );
        REQUIRE( rotor2.e23() == Approx(rotor.e23()) );
        REQUIRE( rotor2.e13() == Approx(rotor.e13()) );
        REQUIRE( rotor2.e12() == Approx(rotor.e12()) );
    }

    SECTION( "translator" )
    {
        auto translator2 = motor.getTranslator();

        REQUIRE( translator2.get<blades::scalar>() == Approx(translator.get<blades::scalar>()) );
        REQUIRE( translator2.get<blades::e1i>() == Approx(translator.get<blades::e1i>()) );
        REQUIRE( translator2.get<blades::e2i>() == Approx(translator.get<blades::e2i>()) );
        REQUIRE( translator2.get<blades::e3i>() == Approx(translator.get<blades::e3i>()) );
    }

    SECTION( "log" )
    {
        auto log = motor.log();

        REQUIRE( log.get<blades::e23>() == Approx(0.0) );
        REQUIRE( log.get<blades::e13>() == Approx(0.0) );
        REQUIRE( log.get<blades::e12>() == Approx(1.5707963266) );
        REQUIRE( log.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( log.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( log.get<blades::e3i>() == Approx(1.0) );
    }

    SECTION( "logJacobian" )
    {
        auto jacobian = motor.logJacobian();

        REQUIRE( jacobian.coeff(0, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 1) == Approx(-2.22144) );
        REQUIRE( jacobian.coeff(0, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(0, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(1, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 2) == Approx(-2.22144) );
        REQUIRE( jacobian.coeff(1, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(1, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(2, 0) == Approx(-0.606986) );
        REQUIRE( jacobian.coeff(2, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 3) == Approx(-2.22144) );
        REQUIRE( jacobian.coeff(2, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(2, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(3, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 1) == Approx(-0.707107) );
        REQUIRE( jacobian.coeff(3, 2) == Approx(0.707107) );
        REQUIRE( jacobian.coeff(3, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 4) == Approx(-1.41421) );
        REQUIRE( jacobian.coeff(3, 5) == Approx(1.41421) );
        REQUIRE( jacobian.coeff(3, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(3, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(4, 0) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 1) == Approx(0.707107) );
        REQUIRE( jacobian.coeff(4, 2) == Approx(0.707107) );
        REQUIRE( jacobian.coeff(4, 3) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 4) == Approx(-1.41421) );
        REQUIRE( jacobian.coeff(4, 5) == Approx(-1.41421) );
        REQUIRE( jacobian.coeff(4, 6) == Approx(0.0) );
        REQUIRE( jacobian.coeff(4, 7) == Approx(0.0) );

        REQUIRE( jacobian.coeff(5, 0) == Approx(0.707107) );
        REQUIRE( jacobian.coeff(5, 1) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 2) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 3) == Approx(-0.707107) );
        REQUIRE( jacobian.coeff(5, 4) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 5) == Approx(0.0) );
        REQUIRE( jacobian.coeff(5, 6) == Approx(-1.41421) );
        REQUIRE( jacobian.coeff(5, 7) == Approx(1.41421) );
    }

    SECTION( "apply to point" )
    {
        SECTION( "origin" )
        {
            Point<double> point;

            Point<double> result = motor.apply(point);

            REQUIRE( result.get<blades::e0>() == Approx(1.0) );
            REQUIRE( result.get<blades::e1>() == Approx(0.0) );
            REQUIRE( result.get<blades::e2>() == Approx(0.0) );
            REQUIRE( result.get<blades::e3>() == Approx(1.0) );
            REQUIRE( result.get<blades::ei>() == Approx(0.5) );
        }

        SECTION( "other" )
        {
            Point<double> point(1.0, 2.0, 3.0);

            Point<double> result = motor.apply(point);

            REQUIRE( result.get<blades::e0>() == Approx(1.0) );
            REQUIRE( result.get<blades::e1>() == Approx(-2.0) );
            REQUIRE( result.get<blades::e2>() == Approx(1.0) );
            REQUIRE( result.get<blades::e3>() == Approx(4.0) );
            REQUIRE( result.get<blades::ei>() == Approx(10.5) );
        }
    }

    SECTION( "apply to line" )
    {
        Point<double> p1;
        Point<double> p2(1.0, 0.0, 0.0);
        Line<double> line(p1, p2);

        Line<double> result = motor.apply(line);

        REQUIRE( result.get<blades::e12i>() == Approx(0.0).margin(1e-6) );
        REQUIRE( result.get<blades::e13i>() == Approx(0.0).margin(1e-6) );
        REQUIRE( result.get<blades::e23i>() == Approx(-1.0) );
        REQUIRE( result.get<blades::e01i>() == Approx(0.0).margin(1e-6) );
        REQUIRE( result.get<blades::e02i>() == Approx(1.0) );
        REQUIRE( result.get<blades::e03i>() == Approx(0.0).margin(1e-6) );
    }

    SECTION( "apply to plane" )
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 0.0, 0.0);
        Point<double> p3(0.0, 1.0, 0.0);
        Plane<double> plane(p1, p2, p3);

        Plane<double> result = motor.apply(plane);

        REQUIRE( result.get<blades::e123i>() == Approx(1.0) );
        REQUIRE( result.get<blades::e012i>() == Approx(plane.get<blades::e012i>()) );
        REQUIRE( result.get<blades::e023i>() == Approx(plane.get<blades::e023i>()) );
        REQUIRE( result.get<blades::e013i>() == Approx(plane.get<blades::e013i>()) );
    }

    SECTION( "apply to sphere" )
    {
        Point<double> center(1.0, 0.0, 0.0);
        Sphere<double> sphere(center, 0.5);

        Sphere<double> result = motor.apply(sphere);

        Point<double> resultCenter = result.getCenter();

        REQUIRE( resultCenter.get<blades::e0>() == Approx(1.0) );
        REQUIRE( resultCenter.get<blades::e1>() == Approx(0.0).margin(1e-6) );
        REQUIRE( resultCenter.get<blades::e2>() == Approx(1.0) );
        REQUIRE( resultCenter.get<blades::e3>() == Approx(1.0) );
        REQUIRE( resultCenter.get<blades::ei>() == Approx(1.0) );

        REQUIRE( result.getRadius() == Approx(sphere.getRadius()) );
    }

    SECTION( "to point pair" )
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 0.0, 0.0);

        PointPair<double> pair(p1, p2);

        PointPair<double> result = motor.apply(pair);

        Point<double> resultPoint1 = result.getPoint1();

        REQUIRE( resultPoint1.get<blades::e1>() == Approx(0.0) );
        REQUIRE( resultPoint1.get<blades::e2>() == Approx(0.0) );
        REQUIRE( resultPoint1.get<blades::e3>() == Approx(1.0) );
        REQUIRE( resultPoint1.get<blades::ei>() == Approx(0.5) );
        REQUIRE( resultPoint1.get<blades::e0>() == Approx(1.0) );

        Point<double> resultPoint2 = result.getPoint2();

        REQUIRE( resultPoint2.get<blades::e1>() == Approx(0.0).margin(1e-6) );
        REQUIRE( resultPoint2.get<blades::e2>() == Approx(1.0) );
        REQUIRE( resultPoint2.get<blades::e3>() == Approx(1.0) );
        REQUIRE( resultPoint2.get<blades::ei>() == Approx(1.0) );
        REQUIRE( resultPoint2.get<blades::e0>() == Approx(1.0) );
    }

    SECTION( "to wrench" )
    {
        Wrench<double> wrench({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

        Wrench<double> result = motor.apply(wrench);

        REQUIRE( result.get<blades::e23>() == Approx(-2.0) );
        REQUIRE( result.get<blades::e13>() == Approx(4.0) );
        REQUIRE( result.get<blades::e12>() == Approx(3.0) );
        REQUIRE( result.get<blades::e01>() == Approx(-5.0) );
        REQUIRE( result.get<blades::e02>() == Approx(4.0) );
        REQUIRE( result.get<blades::e03>() == Approx(6.0) );
    }

    SECTION( "to twist" )
    {
        Twist<double> twist({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

        Twist<double> result = motor.apply(twist);

        REQUIRE( result.get<blades::e23>() == Approx(2.0) );
        REQUIRE( result.get<blades::e13>() == Approx(-1.0) );
        REQUIRE( result.get<blades::e12>() == Approx(3.0) );
        REQUIRE( result.get<blades::e1i>() == Approx(-6.0) );
        REQUIRE( result.get<blades::e2i>() == Approx(6.0) );
        REQUIRE( result.get<blades::e3i>() == Approx(6.0) );
    }
}


TEST_CASE( "Motor with rotation & translation", "[Motor]" )
{
    Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
    Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
    Motor<double> motor(rotor, translator);

    SECTION( "rotor" )
    {
        auto rotor2 = motor.getRotor();

        REQUIRE( rotor2.scalar() == Approx(rotor.scalar()) );
        REQUIRE( rotor2.e23() == Approx(rotor.e23()) );
        REQUIRE( rotor2.e13() == Approx(rotor.e13()) );
        REQUIRE( rotor2.e12() == Approx(rotor.e12()) );
    }

    SECTION( "translator" )
    {
        auto translator2 = motor.getTranslator();

        REQUIRE( translator2.get<blades::scalar>() == Approx(translator.get<blades::scalar>()) );
        REQUIRE( translator2.get<blades::e1i>() == Approx(translator.get<blades::e1i>()) );
        REQUIRE( translator2.get<blades::e2i>() == Approx(translator.get<blades::e2i>()) );
        REQUIRE( translator2.get<blades::e3i>() == Approx(translator.get<blades::e3i>()) );
    }
}


TEST_CASE( "Motor creation from multivector", "[Motor]" )
{
    Multivector<double, blades::scalar, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i, blades::e123i> mv({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0 });

    Motor<double> motor(mv);

    REQUIRE( motor.get<blades::scalar>() == Approx(1.0) );
    REQUIRE( motor.get<blades::e23>() == Approx(2.0) );
    REQUIRE( motor.get<blades::e13>() == Approx(3.0) );
    REQUIRE( motor.get<blades::e12>() == Approx(4.0) );
    REQUIRE( motor.get<blades::e1i>() == Approx(5.0) );
    REQUIRE( motor.get<blades::e2i>() == Approx(6.0) );
    REQUIRE( motor.get<blades::e3i>() == Approx(7.0) );
    REQUIRE( motor.get<blades::e123i>() == Approx(8.0) );
}


TEST_CASE( "Motor creation from generator", "[Motor]" )
{
    Motor<double>::Generator generator({ M_PI / 2.0, 0.0, 0.0, 1.0, 2.0, 3.0 });
    Motor<double> motor(generator);

    SECTION( "rotor" )
    {
        auto rotor = motor.getRotor();

        REQUIRE( rotor.scalar() == Approx(0.707107) );
        REQUIRE( rotor.e23() == Approx(-0.707107) );
        REQUIRE( rotor.e13() == Approx(0.0) );
        REQUIRE( rotor.e12() == Approx(0.0) );
    }

    SECTION( "translator" )
    {
        auto translator = motor.getTranslator();

        REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
        REQUIRE( translator.get<blades::e1i>() == Approx(-0.5) );
        REQUIRE( translator.get<blades::e2i>() == Approx(-1.0) );
        REQUIRE( translator.get<blades::e3i>() == Approx(-1.5) );
    }
}


TEST_CASE( "Motor creation from motor", "[Motor]" )
{
    Motor<double>::Generator generator({ M_PI / 2.0, 0.0, 0.0, 1.0, 2.0, 3.0 });
    Motor<double> motor1(generator);
    Motor<double> motor2(motor1);

    SECTION( "rotor" )
    {
        auto rotor = motor2.getRotor();

        REQUIRE( rotor.scalar() == Approx(0.707107) );
        REQUIRE( rotor.e23() == Approx(-0.707107) );
        REQUIRE( rotor.e13() == Approx(0.0) );
        REQUIRE( rotor.e12() == Approx(0.0) );
    }

    SECTION( "translator" )
    {
        auto translator = motor2.getTranslator();

        REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
        REQUIRE( translator.get<blades::e1i>() == Approx(-0.5) );
        REQUIRE( translator.get<blades::e2i>() == Approx(-1.0) );
        REQUIRE( translator.get<blades::e3i>() == Approx(-1.5) );
    }
}


TEST_CASE( "Motor assignation from motor", "[Motor]" )
{
    Motor<double>::Generator generator({ M_PI / 2.0, 0.0, 0.0, 1.0, 2.0, 3.0 });
    Motor<double> motor1(generator);
    Motor<double> motor2 = motor1;

    SECTION( "rotor" )
    {
        auto rotor = motor2.getRotor();

        REQUIRE( rotor.scalar() == Approx(0.707107) );
        REQUIRE( rotor.e23() == Approx(-0.707107) );
        REQUIRE( rotor.e13() == Approx(0.0) );
        REQUIRE( rotor.e12() == Approx(0.0) );
    }

    SECTION( "translator" )
    {
        auto translator = motor2.getTranslator();

        REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
        REQUIRE( translator.get<blades::e1i>() == Approx(-0.5) );
        REQUIRE( translator.get<blades::e2i>() == Approx(-1.0) );
        REQUIRE( translator.get<blades::e3i>() == Approx(-1.5) );
    }
}


TEST_CASE( "Motor combination", "[Motor]" )
{
    Translator<double>::Generator xTranslationGenerator({ 1.0, 0.0, 0.0 });
    Translator<double>::Generator yTranslationGenerator({ 0.0, 1.0, 0.0 });

    Translator<double> xTranslator(xTranslationGenerator);
    Translator<double> yTranslator(yTranslationGenerator);

    Rotor<double> rotor2({ 0.7071067812, -0.7071067812, 0.0, 0.0 });

    SECTION( "two translations" )
    {
        Motor<double> motor(xTranslator);
        Motor<double> motor2(yTranslator);

        motor *= motor2;

        SECTION( "rotor" )
        {
            auto rotor = motor.getRotor();

            REQUIRE( rotor.scalar() == Approx(1.0) );
            REQUIRE( rotor.e23() == Approx(0.0) );
            REQUIRE( rotor.e13() == Approx(0.0) );
            REQUIRE( rotor.e12() == Approx(0.0) );
        }

        SECTION( "translator" )
        {
            auto translator = motor.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(-0.5) );
            REQUIRE( translator.get<blades::e2i>() == Approx(-0.5) );
            REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
        }
    }

    SECTION( "two rotations" )
    {
        Motor<double> motor(rotor2);
        Motor<double> motor2(rotor2);

        motor *= motor2;

        SECTION( "rotor" )
        {
            auto rotor = motor.getRotor();

            REQUIRE( rotor.scalar() == Approx(0.0) );
            REQUIRE( rotor.e23() == Approx(-1.0) );
            REQUIRE( rotor.e13() == Approx(0.0) );
            REQUIRE( rotor.e12() == Approx(0.0) );
        }

        SECTION( "translator" )
        {
            auto translator = motor.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
        }
    }

    SECTION( "translation times rotation" )
    {
        Motor<double> motor(xTranslator);
        Motor<double> motor2(rotor2);

        motor *= motor2;

        SECTION( "rotor" )
        {
            auto rotor = motor.getRotor();

            REQUIRE( rotor.scalar() == Approx(0.7071067812) );
            REQUIRE( rotor.e23() == Approx(-0.7071067812) );
            REQUIRE( rotor.e13() == Approx(0.0) );
            REQUIRE( rotor.e12() == Approx(0.0) );
        }

        SECTION( "translator" )
        {
            auto translator = motor.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(-0.5) );
            REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
        }
    }

    SECTION( "rotation times translation" )
    {
        Motor<double> motor(rotor2);
        Motor<double> motor2(xTranslator);

        motor *= motor2;

        SECTION( "rotor" )
        {
            auto rotor = motor.getRotor();

            REQUIRE( rotor.scalar() == Approx(0.7071067812) );
            REQUIRE( rotor.e23() == Approx(-0.7071067812) );
            REQUIRE( rotor.e13() == Approx(0.0) );
            REQUIRE( rotor.e12() == Approx(0.0) );
        }

        SECTION( "translator" )
        {
            auto translator = motor.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(-0.5) );
            REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
        }
    }

    SECTION( "complex" )
    {
        Motor<double> motor(xTranslator, rotor2);
        Motor<double> motor2(xTranslator, rotor2);

        motor *= motor2;

        SECTION( "rotor" )
        {
            auto rotor = motor.getRotor();

            REQUIRE( rotor.scalar() == Approx(0.0) );
            REQUIRE( rotor.e23() == Approx(-1.0) );
            REQUIRE( rotor.e13() == Approx(0.0) );
            REQUIRE( rotor.e12() == Approx(0.0) );
        }

        SECTION( "translator" )
        {
            auto translator = motor.getTranslator();

            REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
            REQUIRE( translator.get<blades::e1i>() == Approx(-1.0) );
            REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
            REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
        }
    }
}


TEST_CASE( "Unit Motor creation", "[Motor]" )
{
    auto motor = Motor<double>::Unit();

    SECTION( "rotor" )
    {
        auto rotor = motor.getRotor();

        REQUIRE( rotor.scalar() == Approx(1.0) );
        REQUIRE( rotor.e23() == Approx(0.0) );
        REQUIRE( rotor.e13() == Approx(0.0) );
        REQUIRE( rotor.e12() == Approx(0.0) );
    }

    SECTION( "translator" )
    {
        auto translator = motor.getTranslator();

        REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
        REQUIRE( translator.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
    }
}


TEST_CASE( "Random Motor creation", "[Motor]" )
{
    // We can only check that it compiles and doesn't throw any exception
    auto motor = Motor<double>::Random();

    SECTION( "rotor" )
    {
        REQUIRE_NOTHROW( motor.getRotor() );
    }

    SECTION( "translator" )
    {
        REQUIRE_NOTHROW( motor.getTranslator() );
    }
}


TEST_CASE( "Motor exponential", "[Motor]" )
{
    Motor<double>::Generator generator({ M_PI / 2.0, 0.0, 0.0, 1.0, 2.0, 3.0 });

    auto exp = Motor<double>::exp(generator);

    REQUIRE( exp.get<blades::scalar>() == Approx(0.7071067812) );
    REQUIRE( exp.get<blades::e23>() == Approx(-0.7071067812) );
    REQUIRE( exp.get<blades::e13>() == Approx(0.0) );
    REQUIRE( exp.get<blades::e12>() == Approx(0.0) );
    REQUIRE( exp.get<blades::e1i>() == Approx(-0.3535533906) );
    REQUIRE( exp.get<blades::e2i>() == Approx(-1.767766953) );
    REQUIRE( exp.get<blades::e3i>() == Approx(-0.3535533906) );
    REQUIRE( exp.get<blades::e123i>() == Approx(0.3535533906) );
}
