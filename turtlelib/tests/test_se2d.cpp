#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/se2d.hpp"
#include <sstream>

namespace turtlelib {
    
    TEST_CASE("Twist2D<<", "[se2d]") {
        Twist2D twist = {0, -0.203, 0.145};
        std::stringstream ss{""}; 
        ss << twist;
        REQUIRE(ss.str() == "[0 -0.203 0.145]");
    }

    TEST_CASE("Twist2D>>", "[se2d]") {
        std::stringstream ss("0.1 1.43 -0.2\n"); 
        Twist2D twist;
        ss >> twist;
        REQUIRE_THAT(twist.omega, Catch::Matchers::WithinAbs(0.1, 1.0E-8));
        REQUIRE_THAT(twist.x, Catch::Matchers::WithinAbs(1.43, 1.0E-8));
        REQUIRE_THAT(twist.y, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
        ss = std::stringstream("[0.1 1.43 -0.2]\n"); 
        twist = Twist2D{};
        ss >> twist;
        REQUIRE_THAT(twist.omega, Catch::Matchers::WithinAbs(0.1, 1.0E-8));
        REQUIRE_THAT(twist.x, Catch::Matchers::WithinAbs(1.43, 1.0E-8));
        REQUIRE_THAT(twist.y, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
    }

    TEST_CASE("Transform2D()", "[se2d]") {
        Transform2D tf{};
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    TEST_CASE("Transform2D(trans)", "[se2d]") {
        Vector2D trans{0.2, -0.2};
        Transform2D tf{trans};
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.2, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
    }

    TEST_CASE("Transform2D(rot)", "[se2d]") {
        Transform2D tf{0.1};
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.1, 1.0E-8));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    TEST_CASE("Transform2D(trans, rot)", "[se2d]") {
        Vector2D trans{0.2, -0.2};
        Transform2D tf{trans, 0.1};
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.1, 1.0E-8));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.2, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
    }

    TEST_CASE("Transform2D.translation()", "[se2d]") {
        Vector2D trans{0.2, -0.2};
        Transform2D tf{trans, 0.1};
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.2, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
    }

    TEST_CASE("Transform2D.rotation()", "[se2d]") {
        Vector2D trans{0.2, -0.2};
        Transform2D tf{trans, 0.1};
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.1, 1.0E-8));
    }

    TEST_CASE("Transform2D(point)", "[se2d]") {
        // identity
        Vector2D trans{0.0, 0.0};
        Transform2D tf{trans, 0.0};
        Point2D a{0.3, -0.1};
        Point2D res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.3, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.1, 1.0E-4));

        // pure translation
        trans = {0.2, -0.2};
        tf = Transform2D(trans, 0.0);
        a = {0.3, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.5, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.3, 1.0E-4));

        // pure rotation
        trans = {0.0, 0.0};
        tf = Transform2D(trans, 0.2);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.1179, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.0781, 1.0E-4));

        // rotation & translation
        trans = {0.1, -0.1};
        tf = Transform2D(trans, 3.8);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(-0.0403, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.0821, 1.0E-4));
    }

    TEST_CASE("Transform2D(vector)", "[se2d]") {
        // identity
        Vector2D trans{0.0, 0.0};
        Transform2D tf{trans, 0.0};
        Vector2D a{0.3, -0.1};
        Vector2D res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.3, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.1, 1.0E-4));

        // pure translation
        trans = {0.2, -0.2};
        tf = Transform2D(trans, 0.0);
        a = {0.3, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.3, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.1, 1.0E-4));

        // pure rotation
        trans = {0.0, 0.0};
        tf = Transform2D(trans, 0.2);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.1179, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.0781, 1.0E-4));

        // rotation & translation
        trans = {0.1, -0.1};
        tf = Transform2D(trans, 3.8);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(-0.1403, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(0.0179, 1.0E-4));
    }

    TEST_CASE("Transform2D(twist)", "[se2d]") {
        Vector2D trans{0.2, 1.1};
        Transform2D tf{trans, 0.34};
        Twist2D tw{1.0, 0.707, 0.707};
        Twist2D res{};
        res = tf(tw);
        REQUIRE_THAT(res.omega, Catch::Matchers::WithinAbs(1.0, 1.0E-3));
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(1.531, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(0.702, 1.0E-3));
    }

    TEST_CASE("Transform2D.inv()", "[se2d]") {
        Vector2D trans{1.2, -2.2};
        Transform2D tf{trans, 0.6};
        Transform2D res;
        res = tf.inv();
        REQUIRE_THAT(res.rotation(), Catch::Matchers::WithinAbs(-0.6, 1.0E-3));
        REQUIRE_THAT(res.translation().x, Catch::Matchers::WithinAbs(0.252, 1.0E-3));
        REQUIRE_THAT(res.translation().y, Catch::Matchers::WithinAbs(2.493, 1.0E-3));
    }

    TEST_CASE("Transform2D*=", "[se2d]") {
        // try composition with the inverse
        Vector2D trans{1.2, -2.2};
        Transform2D tf{trans, 0.6};
        Transform2D inv;
        inv = tf.inv();
        tf *= inv;
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-3));

        // try composition with another regular tf
        Vector2D trans1{1.2, -2.2};
        Transform2D tf1{trans1, 0.6};
        Vector2D trans2{0.3, 4.1};
        Transform2D tf2{trans2, -0.1};
        tf1 *= tf2;
        REQUIRE_THAT(tf1.rotation(), Catch::Matchers::WithinAbs(0.5, 1.0E-3));
        REQUIRE_THAT(tf1.translation().x, Catch::Matchers::WithinAbs(-0.867, 1.0E-3));
        REQUIRE_THAT(tf1.translation().y, Catch::Matchers::WithinAbs(1.353, 1.0E-3));
    }

    TEST_CASE("Transform2D*", "[se2d]") {
        // try composition with regular tf
        Vector2D trans1{1.2, -2.2};
        Transform2D tf1{trans1, 0.6};
        Vector2D trans2{0.3, 4.1};
        Transform2D tf2{trans2, -0.1};
        auto tf3 = tf1 * tf2;
        REQUIRE_THAT(tf3.rotation(), Catch::Matchers::WithinAbs(0.5, 1.0E-3));
        REQUIRE_THAT(tf3.translation().x, Catch::Matchers::WithinAbs(-0.867, 1.0E-3));
        REQUIRE_THAT(tf3.translation().y, Catch::Matchers::WithinAbs(1.353, 1.0E-3));
    }

    TEST_CASE("Transform2D<<", "[geometry]") {
        Vector2D trans{1.2, -2.2};
        Transform2D tf{trans, deg2rad(90)};
        std::stringstream ss{""}; 
        ss << tf;
        REQUIRE(ss.str() == "deg: 90 x: 1.2 y: -2.2");
    }

    TEST_CASE("Transform2D>>", "[geometry]") {
        std::stringstream ss("deg: 320 x: -0.2 y: -4.1\n"); 
        Transform2D tf;
        ss >> tf;
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-4.1, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(deg2rad(320), 1.0E-8));
        ss = std::stringstream("320 -0.2 -4.1\n"); 
        tf = Transform2D();
        ss >> tf;
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-4.1, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(deg2rad(320), 1.0E-8));
        ss = std::stringstream("320\n-0.2\n-4.1\n"); 
        tf = Transform2D();
        ss >> tf;
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-4.1, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(deg2rad(320), 1.0E-8));

    }

    TEST_CASE("integrate_twist", "[se2d]") {
        // test only translation
        Twist2D twist{0.0, 1.0, -1.1};
        auto tf = integrate_twist(twist);
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(1.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-1.1, 1.0E-3));
        
        // test only rotation
        twist.omega = 1.0;
        twist.x = 0.0;
        twist.y = 0.0;
        tf = integrate_twist(twist);
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(1.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-3));

        twist.omega = 1.0;
        twist.x = 1.0;
        twist.y = -1.0;
        tf = integrate_twist(twist);
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(1.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(1.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-1.0, 1.0E-3));
    }


}