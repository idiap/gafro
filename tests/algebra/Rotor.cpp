#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;

TEST_CASE("Rotor creation", "[Rotor]")
{
    Rotor<double> rotor;

    REQUIRE(rotor.get<blades::scalar>() == Approx(1.0));
    REQUIRE(rotor.get<blades::e23>() == Approx(0.0));
    REQUIRE(rotor.get<blades::e13>() == Approx(0.0));
    REQUIRE(rotor.get<blades::e12>() == Approx(0.0));
    REQUIRE(rotor.angle() == Approx(0.0));
}

TEST_CASE("Rotor creation from multivector (no angle)", "[Rotor]")
{
    Multivector<double, blades::scalar, blades::e23, blades::e13, blades::e12> mv({ 1.0, 1.0, 0.0, 0.0 });

    Rotor<double> rotor(mv);

    REQUIRE(rotor.get<blades::scalar>() == Approx(1.0));
    REQUIRE(rotor.get<blades::e23>() == Approx(1.0));
    REQUIRE(rotor.get<blades::e13>() == Approx(0.0));
    REQUIRE(rotor.get<blades::e12>() == Approx(0.0));
    REQUIRE(rotor.angle() == Approx(0.0));
}

TEST_CASE("Rotor creation from multivector (with angle)", "[Rotor]")
{
    Multivector<double, blades::scalar, blades::e23, blades::e13, blades::e12> mv({ 0.7071067812, -0.7071067812, 0.0, 0.0 });

    Rotor<double> rotor(mv);

    REQUIRE(rotor.get<blades::scalar>() == Approx(0.7071067812));
    REQUIRE(rotor.get<blades::e23>() == Approx(-0.7071067812));
    REQUIRE(rotor.get<blades::e13>() == Approx(0.0));
    REQUIRE(rotor.get<blades::e12>() == Approx(0.0));
    REQUIRE(rotor.angle() == Approx(M_PI / 2.0));
}

TEST_CASE("Rotor creation from generator", "[Rotor]")
{
    Rotor<double>::Generator generator({ 1.0, 0.0, 0.0 });
    Rotor<double> rotor(generator, M_PI / 2.0);

    REQUIRE(rotor.get<blades::scalar>() == Approx(0.7071067812));
    REQUIRE(rotor.get<blades::e23>() == Approx(-0.7071067812));
    REQUIRE(rotor.get<blades::e13>() == Approx(0.0));
    REQUIRE(rotor.get<blades::e12>() == Approx(0.0));
    REQUIRE(rotor.angle() == Approx(M_PI / 2.0));
}

TEST_CASE("Rotor creation from parameters (no angle)", "[Rotor]")
{
    Rotor<double> rotor({ 1.0, 1.0, 0.0, 0.0 });

    REQUIRE(rotor.get<blades::scalar>() == Approx(1.0));
    REQUIRE(rotor.get<blades::e23>() == Approx(1.0));
    REQUIRE(rotor.get<blades::e13>() == Approx(0.0));
    REQUIRE(rotor.get<blades::e12>() == Approx(0.0));
    REQUIRE(rotor.angle() == Approx(0.0));
}

TEST_CASE("Rotor creation from parameters (with angle)", "[Rotor]")
{
    Rotor<double> rotor({ 0.7071067812, -0.7071067812, 0.0, 0.0 });

    REQUIRE(rotor.get<blades::scalar>() == Approx(0.7071067812));
    REQUIRE(rotor.get<blades::e23>() == Approx(-0.7071067812));
    REQUIRE(rotor.get<blades::e13>() == Approx(0.0));
    REQUIRE(rotor.get<blades::e12>() == Approx(0.0));
    REQUIRE(rotor.angle() == Approx(M_PI / 2.0));
}

TEST_CASE("Rotor creation from expression", "[RotorGenerator]")
{
    Multivector<double, blades::scalar, blades::e23, blades::e13, blades::e12> mv1({ 0.5, -0.5, 0.0, 0.0 });
    Multivector<double, blades::scalar, blades::e23, blades::e13, blades::e12> mv2({ 0.2071067812, -0.2071067812, 0.0, 0.0 });

    Sum<Rotor<double>::Base, Rotor<double>::Base, detail::AdditionOperator> sum(mv1, mv2);

    Rotor<double> rotor(sum);

    REQUIRE(rotor.get<blades::scalar>() == Approx(0.7071067812));
    REQUIRE(rotor.get<blades::e23>() == Approx(-0.7071067812));
    REQUIRE(rotor.get<blades::e13>() == Approx(0.0));
    REQUIRE(rotor.get<blades::e12>() == Approx(0.0));
    REQUIRE(rotor.angle() == Approx(M_PI / 2.0));
}

TEST_CASE("Rotor creation from quaternion", "[Rotor]")
{
    Eigen::Quaternion<double> quaternion(Eigen::AngleAxis(M_PI / 2.0, Eigen::Vector3<double>(1.0, 0.0, 0.0)));

    auto rotor = Rotor<double>::fromQuaternion(quaternion);

    REQUIRE(rotor.get<blades::scalar>() == Approx(0.7071067812));
    REQUIRE(rotor.get<blades::e23>() == Approx(-0.7071067812));
    REQUIRE(rotor.get<blades::e13>() == Approx(0.0));
    REQUIRE(rotor.get<blades::e12>() == Approx(0.0));
    REQUIRE(rotor.angle() == Approx(M_PI / 2.0));
}

TEST_CASE("Rotor get log (no angle)", "[Rotor]")
{
    Rotor<double>::Generator generator({ 1.0, 0.0, 0.0 });
    Rotor<double> rotor(generator, 0.0);

    Rotor<double>::Generator log = rotor.log();

    REQUIRE(log.e23() == Approx(0.0));
    REQUIRE(log.e13() == Approx(0.0));
    REQUIRE(log.e12() == Approx(0.0));
}

TEST_CASE("Rotor get log (with angle)", "[Rotor]")
{
    Rotor<double>::Generator generator({ 1.0, 0.0, 0.0 });
    Rotor<double> rotor(generator, M_PI / 2.0);

    Rotor<double>::Generator log = rotor.log();

    REQUIRE(log.e23() == Approx(M_PI / 2.0));
    REQUIRE(log.e13() == Approx(0.0));
    REQUIRE(log.e12() == Approx(0.0));
}

TEST_CASE("Rotor get quaternion (no angle)", "[Rotor]")
{
    Rotor<double>::Generator generator({ 1.0, 0.0, 0.0 });
    Rotor<double> rotor(generator, 0.0);

    Eigen::Quaternion<double> quaternion = rotor.quaternion();

    REQUIRE(quaternion.w() == Approx(1.0));
    REQUIRE(quaternion.x() == Approx(0.0));
    REQUIRE(quaternion.y() == Approx(0.0));
    REQUIRE(quaternion.z() == Approx(0.0));
}

TEST_CASE("Rotor get quaternion (with angle)", "[Rotor]")
{
    Rotor<double>::Generator generator({ 1.0, 0.0, 0.0 });
    Rotor<double> rotor(generator, M_PI / 2.0);

    Eigen::Quaternion<double> quaternion = rotor.quaternion();

    REQUIRE(quaternion.w() == Approx(0.7071067812));
    REQUIRE(quaternion.x() == Approx(0.7071067812));
    REQUIRE(quaternion.y() == Approx(0.0));
    REQUIRE(quaternion.z() == Approx(0.0));
}

TEST_CASE("Rotor to matrix (no angle)", "[Rotor]")
{
    Rotor<double>::Generator generator({ 1.0, 0.0, 0.0 });
    Rotor<double> rotor(generator, 0.0);

    Eigen::Matrix<double, 3, 3> matrix = rotor.toRotationMatrix();

    REQUIRE(matrix.coeff(0, 0) == Approx(1.0));
    REQUIRE(matrix.coeff(0, 1) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(0, 2) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(1, 0) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(1, 1) == Approx(1.0));
    REQUIRE(matrix.coeff(1, 2) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(2, 0) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(2, 1) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(2, 2) == Approx(1.0));
}

TEST_CASE("Rotor to matrix (with angle)", "[Rotor]")
{
    Rotor<double>::Generator generator({ 1.0, 0.0, 0.0 });
    Rotor<double> rotor(generator, M_PI / 2.0);

    Eigen::Matrix<double, 3, 3> matrix = rotor.toRotationMatrix();

    REQUIRE(matrix.coeff(0, 0) == Approx(1.0));
    REQUIRE(matrix.coeff(0, 1) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(0, 2) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(1, 0) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(1, 1) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(1, 2) == Approx(-1.0));
    REQUIRE(matrix.coeff(2, 0) == Approx(0.0).margin(1e-6));
    REQUIRE(matrix.coeff(2, 1) == Approx(1.0));
    REQUIRE(matrix.coeff(2, 2) == Approx(0.0).margin(1e-6));
}

TEST_CASE("Rotor apply", "[Rotor]")
{
    Rotor<double>::Generator generator({ 0.0, 0.0, 1.0 });
    Rotor<double> rotor(generator, M_PI / 2.0);

    SECTION("to point")
    {
        SECTION("origin")
        {
            Point<double> point;

            Point<double> result = rotor.apply(point);

            REQUIRE(result.get<blades::e0>() == Approx(point.get<blades::e0>()));
            REQUIRE(result.get<blades::e1>() == Approx(point.get<blades::e1>()));
            REQUIRE(result.get<blades::e2>() == Approx(point.get<blades::e2>()));
            REQUIRE(result.get<blades::e3>() == Approx(point.get<blades::e3>()));
            REQUIRE(result.get<blades::ei>() == Approx(point.get<blades::ei>()));
        }

        SECTION("other")
        {
            Point<double> point(1.0, 2.0, 3.0);

            Point<double> result = rotor.apply(point);

            REQUIRE(result.get<blades::e0>() == Approx(1.0));
            REQUIRE(result.get<blades::e1>() == Approx(-2.0));
            REQUIRE(result.get<blades::e2>() == Approx(1.0));
            REQUIRE(result.get<blades::e3>() == Approx(3.0));
            REQUIRE(result.get<blades::ei>() == Approx(7.0));
        }
    }

    SECTION("to line")
    {
        Point<double> p1;
        Point<double> p2(1.0, 0.0, 0.0);
        Line<double> line(p1, p2);

        Line<double> result = rotor.apply(line);

        REQUIRE(result.get<blades::e12i>() == Approx(line.get<blades::e12i>()));
        REQUIRE(result.get<blades::e13i>() == Approx(line.get<blades::e13i>()));
        REQUIRE(result.get<blades::e23i>() == Approx(line.get<blades::e23i>()));
        REQUIRE(result.get<blades::e01i>() == Approx(0.0).margin(1e-6));
        REQUIRE(result.get<blades::e02i>() == Approx(1.0));
        REQUIRE(result.get<blades::e03i>() == Approx(line.get<blades::e03i>()));
    }

    SECTION("to plane")
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 0.0, 0.0);
        Point<double> p3(0.0, 1.0, 0.0);
        Plane<double> plane(p1, p2, p3);

        Plane<double> result = rotor.apply(plane);

        REQUIRE(result.get<blades::e123i>() == Approx(plane.get<blades::e123i>()));
        REQUIRE(result.get<blades::e012i>() == Approx(plane.get<blades::e012i>()));
        REQUIRE(result.get<blades::e023i>() == Approx(plane.get<blades::e023i>()));
        REQUIRE(result.get<blades::e013i>() == Approx(plane.get<blades::e013i>()));
    }

    SECTION("to sphere")
    {
        Point<double> center(1.0, 0.0, 0.0);
        Sphere<double> sphere(center, 0.5);

        Sphere<double> result = rotor.apply(sphere);

        Point<double> resultCenter = result.getCenter();

        REQUIRE(resultCenter.get<blades::e0>() == Approx(center.get<blades::e0>()));
        REQUIRE(resultCenter.get<blades::e1>() == Approx(0.0).margin(1e-6));
        REQUIRE(resultCenter.get<blades::e2>() == Approx(1.0));
        REQUIRE(resultCenter.get<blades::e3>() == Approx(center.get<blades::e3>()));
        REQUIRE(resultCenter.get<blades::ei>() == Approx(center.get<blades::ei>()));

        REQUIRE(result.getRadius() == Approx(sphere.getRadius()));
    }

    SECTION("to point pair")
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 0.0, 0.0);

        PointPair<double> pair(p1, p2);

        PointPair<double> result = rotor.apply(pair);

        Point<double> resultPoint1 = result.getPoint1();

        REQUIRE(resultPoint1.get<blades::e1>() == Approx(0.0));
        REQUIRE(resultPoint1.get<blades::e2>() == Approx(0.0));
        REQUIRE(resultPoint1.get<blades::e3>() == Approx(0.0));
        REQUIRE(resultPoint1.get<blades::ei>() == Approx(0.0));
        REQUIRE(resultPoint1.get<blades::e0>() == Approx(1.0));

        Point<double> resultPoint2 = result.getPoint2();

        REQUIRE(resultPoint2.get<blades::e1>() == Approx(0.0).margin(1e-6));
        REQUIRE(resultPoint2.get<blades::e2>() == Approx(1.0));
        REQUIRE(resultPoint2.get<blades::e3>() == Approx(0.0).margin(1e-6));
        REQUIRE(resultPoint2.get<blades::ei>() == Approx(0.5));
        REQUIRE(resultPoint2.get<blades::e0>() == Approx(1.0));
    }
}

TEST_CASE("Rotor exponential", "[Rotor]")
{
    Rotor<double>::Generator generator({ 1.0, 0.0, 0.0 });

    auto exp = Rotor<double>::exp(generator);

    REQUIRE(exp.get<blades::scalar>() == Approx(0.877583));
    REQUIRE(exp.get<blades::e23>() == Approx(-0.479426));
    REQUIRE(exp.get<blades::e13>() == Approx(0.0));
    REQUIRE(exp.get<blades::e12>() == Approx(0.0));
}
