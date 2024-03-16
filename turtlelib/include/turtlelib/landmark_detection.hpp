#ifndef TURTLELIB_LANDMARK_HPP_INCLUDE_GUARD
#define TURTLELIB_LANDMARK_HPP_INCLUDE_GUARD

#include "turtlelib/geometry2d.hpp"
#include <armadillo>

namespace turtlelib {

    bool checkCircle(const std::vector<turtlelib::Point2D> & cluster);
    std::tuple<double, double, double> fitCircle(arma::mat & cluster);

}

#endif