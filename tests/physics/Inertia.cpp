#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;

TEST_CASE("Inertia default creation", "[Inertia]")
{
    Inertia<double> inertia;

    // Note: Eigen matrices aren't initialised by default, we can't test the values in inertia.getTensor()
}

TEST_CASE("Zero inertia", "[Inertia]")
{
    Inertia<double> inertia = Inertia<double>::Zero();

    Eigen::Matrix<double, 6, 6> tensor = inertia.getTensor();

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j) REQUIRE(tensor.coeff(i, j) == Approx(0.0).margin(1e-6));
    }
}

TEST_CASE("Inertia creation from parameters", "[Inertia]")
{
    Inertia<double> inertia(10.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    SECTION("get tensor")
    {
        Eigen::Matrix<double, 6, 6> tensor2 = inertia.getTensor();

        REQUIRE(tensor2.coeff(0, 0) == Approx(1.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 1) == Approx(1.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 2) == Approx(1.0).margin(1e-6));

        REQUIRE(tensor2.coeff(3, 3) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 4) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 5) == Approx(10.0).margin(1e-6));

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 3; j < 6; ++j)
            {
                REQUIRE(tensor2.coeff(i, j) == Approx(0.0).margin(1e-6));
                REQUIRE(tensor2.coeff(j, i) == Approx(0.0).margin(1e-6));
            }
        }
    }

    SECTION("get elements")
    {
        InertiaElement<double> element = inertia.getElement23();

        REQUIRE(element.get<blades::e23>() == Approx(1.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement13();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(1.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement12();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(1.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement01();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(10.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement02();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(10.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement03();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(10.0).margin(1e-6));
    }
}

TEST_CASE("Inertia creation from tensor", "[Inertia]")
{
    Inertia<double> inertia(10.0, Eigen::Matrix<double, 3, 3>::Identity());

    SECTION("get tensor")
    {
        Eigen::Matrix<double, 6, 6> tensor2 = inertia.getTensor();

        REQUIRE(tensor2.coeff(0, 0) == Approx(1.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 1) == Approx(1.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 2) == Approx(1.0).margin(1e-6));

        REQUIRE(tensor2.coeff(3, 3) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 4) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 5) == Approx(10.0).margin(1e-6));

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 3; j < 6; ++j)
            {
                REQUIRE(tensor2.coeff(i, j) == Approx(0.0).margin(1e-6));
                REQUIRE(tensor2.coeff(j, i) == Approx(0.0).margin(1e-6));
            }
        }
    }

    SECTION("get elements")
    {
        InertiaElement<double> element = inertia.getElement23();

        REQUIRE(element.get<blades::e23>() == Approx(1.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement13();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(1.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement12();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(1.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement01();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(10.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement02();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(10.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement03();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(10.0).margin(1e-6));
    }
}

TEST_CASE("Inertia creation from elements", "[Inertia]")
{
    std::array<InertiaElement<double>, 6> elements;
    elements[0] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    elements[1] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 };
    elements[2] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
    elements[3] = { 0.0, 0.0, 0.0, 10.0, 0.0, 0.0 };
    elements[4] = { 0.0, 0.0, 0.0, 0.0, 10.0, 0.0 };
    elements[5] = { 0.0, 0.0, 0.0, 0.0, 0.0, 10.0 };

    Inertia<double> inertia(elements);

    SECTION("get tensor")
    {
        Eigen::Matrix<double, 6, 6> tensor2 = inertia.getTensor();

        REQUIRE(tensor2.coeff(0, 0) == Approx(1.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 1) == Approx(1.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 2) == Approx(1.0).margin(1e-6));

        REQUIRE(tensor2.coeff(3, 3) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 4) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 5) == Approx(10.0).margin(1e-6));

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 3; j < 6; ++j)
            {
                REQUIRE(tensor2.coeff(i, j) == Approx(0.0).margin(1e-6));
                REQUIRE(tensor2.coeff(j, i) == Approx(0.0).margin(1e-6));
            }
        }
    }

    SECTION("get elements")
    {
        InertiaElement<double> element = inertia.getElement23();

        REQUIRE(element.get<blades::e23>() == Approx(1.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement13();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(1.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement12();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(1.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement01();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(10.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement02();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(10.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(0.0).margin(1e-6));

        element = inertia.getElement03();

        REQUIRE(element.get<blades::e23>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e13>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e12>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e01>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e02>() == Approx(0.0).margin(1e-6));
        REQUIRE(element.get<blades::e03>() == Approx(10.0).margin(1e-6));
    }
}

TEST_CASE("Inertias addition", "[Inertia]")
{
    Inertia<double> inertia(10.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    Inertia<double> inertia2(20.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0);

    SECTION("in-place operator")
    {
        inertia += inertia2;

        Eigen::Matrix<double, 6, 6> tensor = inertia.getTensor();

        REQUIRE(tensor.coeff(0, 0) == Approx(11.0));
        REQUIRE(tensor.coeff(0, 1) == Approx(-22.0));
        REQUIRE(tensor.coeff(0, 2) == Approx(33.0));
        REQUIRE(tensor.coeff(1, 0) == Approx(-22.0));
        REQUIRE(tensor.coeff(1, 1) == Approx(44.0));
        REQUIRE(tensor.coeff(1, 2) == Approx(-55.0));
        REQUIRE(tensor.coeff(2, 0) == Approx(33.0));
        REQUIRE(tensor.coeff(2, 1) == Approx(-55.0));
        REQUIRE(tensor.coeff(2, 2) == Approx(66.0));

        REQUIRE(tensor.coeff(3, 3) == Approx(30.0).margin(1e-6));
        REQUIRE(tensor.coeff(3, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(3, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(4, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(4, 4) == Approx(30.0).margin(1e-6));
        REQUIRE(tensor.coeff(4, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(5, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(5, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(5, 5) == Approx(30.0).margin(1e-6));

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 3; j < 6; ++j)
            {
                REQUIRE(tensor.coeff(i, j) == Approx(0.0).margin(1e-6));
                REQUIRE(tensor.coeff(j, i) == Approx(0.0).margin(1e-6));
            }
        }
    }

    SECTION("binary operator")
    {
        Inertia<double> inertia3 = inertia + inertia2;

        Eigen::Matrix<double, 6, 6> tensor = inertia3.getTensor();

        REQUIRE(tensor.coeff(0, 0) == Approx(11.0));
        REQUIRE(tensor.coeff(0, 1) == Approx(-22.0));
        REQUIRE(tensor.coeff(0, 2) == Approx(33.0));
        REQUIRE(tensor.coeff(1, 0) == Approx(-22.0));
        REQUIRE(tensor.coeff(1, 1) == Approx(44.0));
        REQUIRE(tensor.coeff(1, 2) == Approx(-55.0));
        REQUIRE(tensor.coeff(2, 0) == Approx(33.0));
        REQUIRE(tensor.coeff(2, 1) == Approx(-55.0));
        REQUIRE(tensor.coeff(2, 2) == Approx(66.0));

        REQUIRE(tensor.coeff(3, 3) == Approx(30.0).margin(1e-6));
        REQUIRE(tensor.coeff(3, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(3, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(4, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(4, 4) == Approx(30.0).margin(1e-6));
        REQUIRE(tensor.coeff(4, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(5, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(5, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor.coeff(5, 5) == Approx(30.0).margin(1e-6));

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 3; j < 6; ++j)
            {
                REQUIRE(tensor.coeff(i, j) == Approx(0.0).margin(1e-6));
                REQUIRE(tensor.coeff(j, i) == Approx(0.0).margin(1e-6));
            }
        }
    }
}

TEST_CASE("Inertia operator", "[Inertia]")
{
    Inertia<double> inertia(10.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    Twist<double> twist({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    Wrench<double> result = inertia(twist);

    REQUIRE(result.get<blades::e23>() == Approx(1.0));
    REQUIRE(result.get<blades::e13>() == Approx(2.0));
    REQUIRE(result.get<blades::e12>() == Approx(3.0));
    REQUIRE(result.get<blades::e01>() == Approx(40.0));
    REQUIRE(result.get<blades::e02>() == Approx(50.0));
    REQUIRE(result.get<blades::e03>() == Approx(60.0));
}

TEST_CASE("Inertia transforms", "[Inertia]")
{
    Inertia<double> inertia(10.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    SECTION("transform")
    {
        Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
        Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
        Motor<double> motor(translator, rotor);

        Inertia<double> result = inertia.transform(motor);

        Eigen::Matrix<double, 6, 6> tensor2 = result.getTensor();

        REQUIRE(tensor2.coeff(0, 0) == Approx(14.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 1) == Approx(2.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 2) == Approx(-5.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 0) == Approx(2.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 1) == Approx(11.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 2) == Approx(-3.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 0) == Approx(-5.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 1) == Approx(-3.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 2) == Approx(6.0).margin(1e-6));

        REQUIRE(tensor2.coeff(0, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 4) == Approx(-10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 3) == Approx(-10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 5) == Approx(0.0).margin(1e-6));

        REQUIRE(tensor2.coeff(3, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 1) == Approx(-10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 0) == Approx(-10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 2) == Approx(0.0).margin(1e-6));

        REQUIRE(tensor2.coeff(3, 3) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 4) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 5) == Approx(10.0).margin(1e-6));
    }

    SECTION("inverse transform")
    {
        Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
        Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
        Motor<double> motor(translator, rotor);

        Inertia<double> result = inertia.inverseTransform(motor);

        Eigen::Matrix<double, 6, 6> tensor2 = result.getTensor();

        REQUIRE(tensor2.coeff(0, 0) == Approx(14.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 1) == Approx(2.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 2) == Approx(5.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 0) == Approx(2.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 1) == Approx(11.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 2) == Approx(3.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 0) == Approx(5.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 1) == Approx(3.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 2) == Approx(6.0).margin(1e-6));

        REQUIRE(tensor2.coeff(0, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 4) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 3) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 5) == Approx(0.0).margin(1e-6));

        REQUIRE(tensor2.coeff(3, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 1) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 0) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 2) == Approx(0.0).margin(1e-6));

        REQUIRE(tensor2.coeff(3, 3) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 4) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 5) == Approx(10.0).margin(1e-6));
    }
}