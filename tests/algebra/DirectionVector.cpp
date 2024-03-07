#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "DirectionVector creation", "[DirectionVector]" )
{
    DirectionVector<double> vector;

    REQUIRE( vector.get<blades::e1i>() == Approx(0.0) );
    REQUIRE( vector.get<blades::e2i>() == Approx(0.0) );
    REQUIRE( vector.get<blades::e3i>() == Approx(0.0) );
}


TEST_CASE( "DirectionVector creation from parameters", "[DirectionVector]" )
{
    DirectionVector<double> vector(1.0, 2.0, 3.0);

    REQUIRE( vector.get<blades::e1i>() == Approx(1.0) );
    REQUIRE( vector.get<blades::e2i>() == Approx(2.0) );
    REQUIRE( vector.get<blades::e3i>() == Approx(3.0) );
}


TEST_CASE( "DirectionVector creation from direciton vector", "[DirectionVector]" )
{
    DirectionVector<double> vector1(1.0, 2.0, 3.0);
    DirectionVector<double> vector2(vector1);

    REQUIRE( vector2.get<blades::e1i>() == Approx(1.0) );
    REQUIRE( vector2.get<blades::e2i>() == Approx(2.0) );
    REQUIRE( vector2.get<blades::e3i>() == Approx(3.0) );
}


TEST_CASE( "DirectionVector creation from Multivector", "[DirectionVector]" )
{
    Multivector<double, blades::e1i, blades::e2i, blades::e3i> mv({ 1.0, 2.0, 3.0 });
    DirectionVector<double> vector(mv);

    REQUIRE( vector.get<blades::e1i>() == Approx(1.0) );
    REQUIRE( vector.get<blades::e2i>() == Approx(2.0) );
    REQUIRE( vector.get<blades::e3i>() == Approx(3.0) );
}


TEST_CASE( "DirectionVector creation from expression", "[DirectionVector]" )
{

    DirectionVector<double> vector1( 1.0, 2.0, 3.0 );
    DirectionVector<double> vector2( 10.0, 20.0, 30.0 );

    Sum<DirectionVector<double>, DirectionVector<double>, detail::AdditionOperator> sum(vector1, vector2);

    DirectionVector<double> vector(sum);

    REQUIRE( vector.get<blades::e1i>() == Approx(11.0) );
    REQUIRE( vector.get<blades::e2i>() == Approx(22.0) );
    REQUIRE( vector.get<blades::e3i>() == Approx(33.0) );
}
