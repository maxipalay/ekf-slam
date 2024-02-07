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

    TEST_CASE("Normalize(Vector2D)", "[geometry]") {
        Vector2D vec{1.0,1.0};
        Vector2D normalized = normalize(vec);
        REQUIRE_THAT(normalized.x, Catch::Matchers::WithinAbs(0.707, 1.0E-3));
        REQUIRE_THAT(normalized.y, Catch::Matchers::WithinAbs(0.707, 1.0E-3));
    }

    TEST_CASE("Vector2D+", "[geometry]") {
        Vector2D vec1{1.0,1.0};
        Vector2D vec2{-1.0,1.0};
        auto res = vec1 + vec2;
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.0, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(2.0, 1.0E-3));
    }

    TEST_CASE("Vector2D-", "[geometry]") {
        Vector2D vec1{1.0,1.0};
        Vector2D vec2{-1.0,0.0};
        auto res = vec1 - vec2;
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(2.0, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(1.0, 1.0E-3));
    }

    TEST_CASE("Vector2D*scalar", "[geometry]") {
        Vector2D vec1{1.0,0.0};
        double factor{0.1};
        auto res = vec1 * factor;
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.1, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(0.0, 1.0E-3));
    }

    TEST_CASE("scalar*Vector2D", "[geometry]") {
        Vector2D vec1{1.0,0.0};
        double factor{0.1};
        auto res = factor * vec1;
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.1, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(0.0, 1.0E-3));
    }

    TEST_CASE("Vector2D+=", "[geometry]") {
        Vector2D vec1{1.0,1.0};
        Vector2D vec2{-1.0,1.0};
        vec1 += vec2;
        REQUIRE_THAT(vec1.x, Catch::Matchers::WithinAbs(0.0, 1.0E-3));
        REQUIRE_THAT(vec1.y, Catch::Matchers::WithinAbs(2.0, 1.0E-3));
    }

    TEST_CASE("Vector2D-=", "[geometry]") {
        Vector2D vec1{1.0,1.0};
        Vector2D vec2{-1.0,0.0};
        vec1 -= vec2;
        REQUIRE_THAT(vec1.x, Catch::Matchers::WithinAbs(2.0, 1.0E-3));
        REQUIRE_THAT(vec1.y, Catch::Matchers::WithinAbs(1.0, 1.0E-3));
    }

    TEST_CASE("Vector2D*=", "[geometry]") {
        Vector2D vec1{1.0,0.0};
        double scalar{0.1};
        vec1 *= scalar;
        REQUIRE_THAT(vec1.x, Catch::Matchers::WithinAbs(0.1, 1.0E-3));
        REQUIRE_THAT(vec1.y, Catch::Matchers::WithinAbs(0.0, 1.0E-3));
    }

    TEST_CASE("dot", "[geometry]") {
        Vector2D vec1{1.0,0.0};
        Vector2D vec2{-1.0,-1.0};
        auto res = dot(vec1, vec2);
        REQUIRE_THAT(res, Catch::Matchers::WithinAbs(-1.0, 1.0E-3));
    }

    TEST_CASE("magnitude", "[geometry]") {
        Vector2D vec1{1.0,0.0};
        Vector2D vec2{-1.0,-1.0};
        auto res1 = magnitude(vec1);
        auto res2 = magnitude(vec2);
        REQUIRE_THAT(res1, Catch::Matchers::WithinAbs(1.0, 1.0E-3));
        REQUIRE_THAT(res2, Catch::Matchers::WithinAbs(1.414, 1.0E-3));
    }

    TEST_CASE("angle", "[geometry]") {
        Vector2D vec1{1.0,0.0};
        Vector2D vec2{-1.0,-1.0};
        auto ang = angle(vec1, vec2);
        REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(3.0*PI/4.0, 1.0E-3));
        
    }

    TEST_CASE("angle diff", "[geometry]") {
        double angle1{3.0*PI/4.0};
        double angle2{-3.0*PI/4.0};
        auto ang = angle_diff(angle1, angle2);
        REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(2.0*PI/4.0, 1.0E-3));

        ang = angle_diff(angle2, angle1);
        REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(-2.0*PI/4.0, 1.0E-3));

        angle1 = PI/4.0;
        angle2 = -PI/4.0;
        ang = angle_diff(angle1, angle2);
        REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(-2.0*PI/4.0, 1.0E-3));

        ang = angle_diff(angle2, angle1);
        REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(2.0*PI/4.0, 1.0E-3));

        angle1 = PI/4.0;
        angle2 = PI/2.0;
        ang = angle_diff(angle1, angle2);
        REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(PI/4.0, 1.0E-3));

        ang = angle_diff(angle2, angle1);
        REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(-PI/4.0, 1.0E-3));

        angle1 = -PI/4.0;
        angle2 = -PI/2.0;
        ang = angle_diff(angle1, angle2);
        REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(-PI/4.0, 1.0E-3));

        ang = angle_diff(angle2, angle1);
        REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(PI/4.0, 1.0E-3));

        
    }

}

