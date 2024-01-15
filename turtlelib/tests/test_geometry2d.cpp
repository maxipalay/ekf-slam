#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/geometry2d.hpp"
#include <sstream>

namespace turtlelib {

    TEST_CASE("Normalize angles", "[geometry]") {
        REQUIRE_THAT(normalize_angle(0), Catch::Matchers::WithinAbs(0, 1.0E-8));
        REQUIRE_THAT(normalize_angle(PI), Catch::Matchers::WithinAbs(PI, 1.0E-8));
        REQUIRE_THAT(normalize_angle(-PI), Catch::Matchers::WithinAbs(PI, 1.0E-8));
        REQUIRE_THAT(normalize_angle(-PI/4), Catch::Matchers::WithinAbs(-PI/4, 1.0E-8));
        REQUIRE_THAT(normalize_angle(3*PI/2), Catch::Matchers::WithinAbs(-PI/2, 1.0E-8));
        REQUIRE_THAT(normalize_angle(-3*PI/2), Catch::Matchers::WithinAbs(PI/2, 1.0E-8));
        REQUIRE_THAT(normalize_angle(-5*PI/2), Catch::Matchers::WithinAbs(-PI/2, 1.0E-8));
        REQUIRE_THAT(normalize_angle(5*PI/2), Catch::Matchers::WithinAbs(PI/2, 1.0E-8));
    }

    TEST_CASE("Point2D<<", "[geometry]") {
        Point2D point = {0, -203};
        std::stringstream ss{""}; 
        ss << point;
        REQUIRE(ss.str() == "[0 -203]");
    }

    TEST_CASE("Point2D>>", "[geometry]") {
        std::stringstream ss("-2 1243\n"); 
        Point2D point;
        ss >> point;
        REQUIRE_THAT(point.x, Catch::Matchers::WithinAbs(-2.0, 1.0E-8));
        REQUIRE_THAT(point.y, Catch::Matchers::WithinAbs(1243.0, 1.0E-8));
        ss = std::stringstream("[-2 0]\n"); 
        point = Point2D{};
        ss >> point;
        REQUIRE_THAT(point.x, Catch::Matchers::WithinAbs(-2.0, 1.0E-8));
        REQUIRE_THAT(point.y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    TEST_CASE("Point2D-Point2D", "[geometry]") {
        Point2D point_a = {0, -203};
        Point2D point_b = {1, -1};
        Vector2D expected_res = {-1, -202};
        Vector2D res = point_a - point_b;
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(expected_res.x, 1.0E-8));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(expected_res.y, 1.0E-8));
    }

    TEST_CASE("Point2D+Vector2D", "[geometry]") {
        Point2D p = {0, -203};
        Vector2D v = {1, -1};
        Point2D expected_res = {1, -204};
        Point2D res = p + v;
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(expected_res.x, 1.0E-8));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(expected_res.y, 1.0E-8));
    }

    TEST_CASE("Vector2D<<", "[geometry]") {
        Vector2D vec = {0, -203};
        std::stringstream ss{""}; 
        ss << vec;
        REQUIRE(ss.str() == "[0 -203]");
    }

    TEST_CASE("Vector2D>>", "[geometry]") {
        std::stringstream ss("-2 1243\n"); 
        Vector2D vec;
        ss >> vec;
        REQUIRE_THAT(vec.x, Catch::Matchers::WithinAbs(-2.0, 1.0E-8));
        REQUIRE_THAT(vec.y, Catch::Matchers::WithinAbs(1243.0, 1.0E-8));
        ss = std::stringstream("[-2 0]\n"); 
        vec = Vector2D{};
        ss >> vec;
        REQUIRE_THAT(vec.x, Catch::Matchers::WithinAbs(-2.0, 1.0E-8));
        REQUIRE_THAT(vec.y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

}

