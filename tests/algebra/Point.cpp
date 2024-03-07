#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Point creation", "[Point]" )
{
    Point<double> point;

    REQUIRE( point.get<blades::e1>() == Approx(0.0) );
    REQUIRE( point.get<blades::e2>() == Approx(0.0) );
    REQUIRE( point.get<blades::e3>() == Approx(0.0) );
    REQUIRE( point.get<blades::ei>() == Approx(0.0) );
    REQUIRE( point.get<blades::e0>() == Approx(1.0) );
}


TEST_CASE( "Point creation from parameters", "[Point]" )
{
    Point<double> point(1.0, 2.0, 3.0);

    REQUIRE( point.get<blades::e1>() == Approx(1.0) );
    REQUIRE( point.get<blades::e2>() == Approx(2.0) );
    REQUIRE( point.get<blades::e3>() == Approx(3.0) );
    REQUIRE( point.get<blades::ei>() == Approx(7.0) );
    REQUIRE( point.get<blades::e0>() == Approx(1.0) );
}


TEST_CASE( "Point creation from point", "[Point]" )
{
    Point<double> point1(1.0, 2.0, 3.0);
    Point<double> point2(point1);

    REQUIRE( point2.get<blades::e1>() == Approx(1.0) );
    REQUIRE( point2.get<blades::e2>() == Approx(2.0) );
    REQUIRE( point2.get<blades::e3>() == Approx(3.0) );
    REQUIRE( point2.get<blades::ei>() == Approx(7.0) );
    REQUIRE( point2.get<blades::e0>() == Approx(1.0) );
}


TEST_CASE( "Point creation from Multivector", "[Point]" )
{
    Multivector<double, blades::e1, blades::e2, blades::e3, blades::ei, blades::e0> mv({ 1.0, 2.0, 3.0, 7.0, 1.0 });
    Point<double> point(mv);

    REQUIRE( point.get<blades::e1>() == Approx(1.0) );
    REQUIRE( point.get<blades::e2>() == Approx(2.0) );
    REQUIRE( point.get<blades::e3>() == Approx(3.0) );
    REQUIRE( point.get<blades::ei>() == Approx(7.0) );
    REQUIRE( point.get<blades::e0>() == Approx(1.0) );
}


TEST_CASE( "Point creation from expression", "[Point]" )
{

    Point<double> point1( 1.0, 2.0, 3.0 );
    Point<double> point2( 10.0, 20.0, 30.0 );

    Sum<Point<double>, Point<double>, detail::AdditionOperator> sum(point1, point2);

    Point<double> point(sum);

    REQUIRE( point.get<blades::e1>() == Approx(11.0) );
    REQUIRE( point.get<blades::e2>() == Approx(22.0) );
    REQUIRE( point.get<blades::e3>() == Approx(33.0) );
    REQUIRE( point.get<blades::ei>() == Approx(707.0) );
    REQUIRE( point.get<blades::e0>() == Approx(2.0) );
}
