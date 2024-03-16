#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/landmark_detection.hpp"
#include <armadillo>

namespace turtlelib {
    
    TEST_CASE("Circle fitting - test 1", "[landmark]") {
        arma::mat points = {{1,7},{2,6},{5,8},{7,7},{9,5},{3,7}};

        double c_x;
      double c_y;
      double r;
      std::tie(c_x, c_y, r) = fitCircle(points);


        REQUIRE_THAT(c_x, Catch::Matchers::WithinAbs(4.615482, 1.0E-4));
        REQUIRE_THAT(c_y, Catch::Matchers::WithinAbs(2.807354, 1.0E-4));
        REQUIRE_THAT(r, Catch::Matchers::WithinAbs(4.8275, 1.0E-4));
    }

    TEST_CASE("Circle fitting - test 2", "[landmark]") {
        arma::mat points = {{-1,0},{-0.3,-0.06},{0.3,0.1},{1,0}};

        double c_x;
      double c_y;
      double r;
      std::tie(c_x, c_y, r) = fitCircle(points);


        REQUIRE_THAT(c_x, Catch::Matchers::WithinAbs(0.4908357, 1.0E-4));
        REQUIRE_THAT(c_y, Catch::Matchers::WithinAbs(-22.15212, 1.0E-4));
        REQUIRE_THAT(r, Catch::Matchers::WithinAbs(22.17979, 1.0E-4));
    }

}