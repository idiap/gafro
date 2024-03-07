#include <catch.hpp>
#include <gafro/gafro.hpp>
#include <cmath>

using namespace gafro;


TEST_CASE( "PointPair creation", "[PointPair]" )
{
    PointPair<double> pair;

    REQUIRE( pair.get<blades::e23>() == Approx(0.0) );
    REQUIRE( pair.get<blades::e13>() == Approx(0.0) );
    REQUIRE( pair.get<blades::e12>() == Approx(0.0) );
    REQUIRE( pair.get<blades::e1i>() == Approx(0.0) );
    REQUIRE( pair.get<blades::e2i>() == Approx(0.0) );
    REQUIRE( pair.get<blades::e3i>() == Approx(0.0) );
    REQUIRE( pair.get<blades::e01>() == Approx(0.0) );
    REQUIRE( pair.get<blades::e02>() == Approx(0.0) );
    REQUIRE( pair.get<blades::e03>() == Approx(0.0) );
    REQUIRE( pair.get<blades::e0i>() == Approx(0.0) );

    Point<double> p1 = pair.getPoint1();

    REQUIRE( std::isnan(p1.get<blades::e1>()) );
    REQUIRE( std::isnan(p1.get<blades::e2>()) );
    REQUIRE( std::isnan(p1.get<blades::e3>()) );
    REQUIRE( std::isnan(p1.get<blades::ei>()) );
    REQUIRE( std::isnan(p1.get<blades::e0>()) );

    Point<double> p2 = pair.getPoint2();

    REQUIRE( std::isnan(p2.get<blades::e1>()) );
    REQUIRE( std::isnan(p2.get<blades::e2>()) );
    REQUIRE( std::isnan(p2.get<blades::e3>()) );
    REQUIRE( std::isnan(p2.get<blades::ei>()) );
    REQUIRE( std::isnan(p2.get<blades::e0>()) );
}


TEST_CASE( "PointPair creation from points", "[PointPair]" )
{
    Point<double> p1(1.0, 2.0, 3.0);
    Point<double> p2(4.0, 5.0, 6.0);

    PointPair<double> pair(p1, p2);

    REQUIRE( pair.get<blades::e23>() == Approx(-3.0) );
    REQUIRE( pair.get<blades::e13>() == Approx(-6.0) );
    REQUIRE( pair.get<blades::e12>() == Approx(-3.0) );
    REQUIRE( pair.get<blades::e1i>() == Approx(10.5) );
    REQUIRE( pair.get<blades::e2i>() == Approx(42.0) );
    REQUIRE( pair.get<blades::e3i>() == Approx(73.5) );
    REQUIRE( pair.get<blades::e01>() == Approx(3.0) );
    REQUIRE( pair.get<blades::e02>() == Approx(3.0) );
    REQUIRE( pair.get<blades::e03>() == Approx(3.0) );
    REQUIRE( pair.get<blades::e0i>() == Approx(31.5) );

    Point<double> p1b = pair.getPoint1();

    REQUIRE( p1b.get<blades::e1>() == Approx(p1.get<blades::e1>()) );
    REQUIRE( p1b.get<blades::e2>() == Approx(p1.get<blades::e2>()) );
    REQUIRE( p1b.get<blades::e3>() == Approx(p1.get<blades::e3>()) );
    REQUIRE( p1b.get<blades::ei>() == Approx(p1.get<blades::ei>()) );
    REQUIRE( p1b.get<blades::e0>() == Approx(p1.get<blades::e0>()) );

    Point<double> p2b = pair.getPoint2();

    REQUIRE( p2b.get<blades::e1>() == Approx(p2.get<blades::e1>()) );
    REQUIRE( p2b.get<blades::e2>() == Approx(p2.get<blades::e2>()) );
    REQUIRE( p2b.get<blades::e3>() == Approx(p2.get<blades::e3>()) );
    REQUIRE( p2b.get<blades::ei>() == Approx(p2.get<blades::ei>()) );
    REQUIRE( p2b.get<blades::e0>() == Approx(p2.get<blades::e0>()) );
}


TEST_CASE( "PointPair creation from pair", "[PointPair]" )
{
    Point<double> p1(1.0, 2.0, 3.0);
    Point<double> p2(4.0, 5.0, 6.0);

    PointPair<double> pair(p1, p2);
    PointPair<double> pair2(pair);

    REQUIRE( pair2.get<blades::e23>() == Approx(-3.0) );
    REQUIRE( pair2.get<blades::e13>() == Approx(-6.0) );
    REQUIRE( pair2.get<blades::e12>() == Approx(-3.0) );
    REQUIRE( pair2.get<blades::e1i>() == Approx(10.5) );
    REQUIRE( pair2.get<blades::e2i>() == Approx(42.0) );
    REQUIRE( pair2.get<blades::e3i>() == Approx(73.5) );
    REQUIRE( pair2.get<blades::e01>() == Approx(3.0) );
    REQUIRE( pair2.get<blades::e02>() == Approx(3.0) );
    REQUIRE( pair2.get<blades::e03>() == Approx(3.0) );
    REQUIRE( pair2.get<blades::e0i>() == Approx(31.5) );
}


TEST_CASE( "PointPair creation from Multivector", "[PointPair]" )
{
    Multivector<double, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i, blades::e01, blades::e02,
                        blades::e03, blades::e0i> mv({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0 });
    PointPair<double> pair(mv);

    REQUIRE( pair.get<blades::e23>() == Approx(1.0) );
    REQUIRE( pair.get<blades::e13>() == Approx(2.0) );
    REQUIRE( pair.get<blades::e12>() == Approx(3.0) );
    REQUIRE( pair.get<blades::e1i>() == Approx(4.0) );
    REQUIRE( pair.get<blades::e2i>() == Approx(5.0) );
    REQUIRE( pair.get<blades::e3i>() == Approx(6.0) );
    REQUIRE( pair.get<blades::e01>() == Approx(7.0) );
    REQUIRE( pair.get<blades::e02>() == Approx(8.0) );
    REQUIRE( pair.get<blades::e03>() == Approx(9.0) );
    REQUIRE( pair.get<blades::e0i>() == Approx(10.0) );
}
