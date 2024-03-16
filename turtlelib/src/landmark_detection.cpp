#include "turtlelib/landmark_detection.hpp"

namespace turtlelib {

  bool checkCircle(const std::vector<turtlelib::Point2D> & cluster)
  {
    // classify as circle/not circle
    arma::vec angles = arma::vec(cluster.size() - 2);

    turtlelib::Point2D cluster_start = cluster.at(0);
    turtlelib::Point2D cluster_end = cluster.at(cluster.size() - 1);
    // for each intermediate point
    for (size_t j = 1; j < cluster.size() - 1; j++) {

      //
      double a = std::sqrt(
        std::pow(
          cluster_start.x - cluster.at(
            j).x, 2) + std::pow(cluster_start.y - cluster.at(j).y, 2));
      double b =
        std::sqrt(
        std::pow(
          cluster_end.x -
          cluster.at(j).x,
          2) + std::pow(
          cluster_end.y - cluster.at(
            j).y, 2));
      double c =
        std::sqrt(
        std::pow(
          cluster_start.x -
          cluster_end.x,
          2) + std::pow(
          cluster_start.y - cluster_end.y, 2));


      double angle = std::acos((c * c - a * a - b * b) / (-2.0 * a * b));
      angles(j - 1) = angle;
    }

    double stdev = arma::stddev(angles);
    double mn = arma::mean(angles);
    
    if (stdev < 0.2 && 1.3 < mn && mn < 2.6) {
      return true;
    }
    return false;
  }

  std::tuple<double, double, double> fitCircle(arma::mat & cluster)
  {
    // subtract the means
    arma::mat means = arma::mean(cluster, 0);     // calculate the means across the cols (x,y)
    arma::vec offset_x =
      arma::vec(arma::size(cluster)(0), arma::fill::ones) * means(0);
    arma::vec offset_y =
      arma::vec(arma::size(cluster)(0), arma::fill::ones) * means(1);
    cluster.col(0) = cluster.col(0) - offset_x;
    cluster.col(1) = cluster.col(1) - offset_y;
    arma::vec z = arma::pow(cluster.col(0), 2) + arma::pow(
      cluster.col(1), 2);
    auto z_mean = arma::mean(z);
    arma::mat Z = arma::mat(arma::size(cluster)(0), 4);

    Z.col(0) = z;
    Z.col(1) = cluster.col(0);
    Z.col(2) = cluster.col(1);
    Z.col(3) = arma::vec(arma::size(cluster)(0), arma::fill::ones);

    auto M = 1.0 / static_cast<double>(arma::size(cluster)(0)) * Z.t() * Z;
    auto H = arma::mat(4, 4, arma::fill::eye);
    H(0, 0) = 8.0 * z_mean;
    H(3, 3) = 0.0;
    H(3, 0) = 2.0;
    H(0, 3) = 2.0;

    auto H_inv = arma::mat(4, 4, arma::fill::eye);
    H_inv(0, 0) = 0.0;
    H_inv(3, 0) = 0.5;
    H_inv(0, 3) = 0.5;
    H_inv(3, 3) = -2.0 * z_mean;

    //   RCLCPP_INFO_STREAM(get_logger(), "H_inv " << arma::size(H_inv) << std::endl);

    arma::mat U;
    arma::vec s;
    arma::mat V;

    arma::svd(U, s, V, Z);
    //   RCLCPP_INFO_STREAM(get_logger(), "U " << arma::size(U) << std::endl);
    //   RCLCPP_INFO_STREAM(get_logger(), "s " << arma::size(s) << std::endl);
    //   RCLCPP_INFO_STREAM(get_logger(), "V " << arma::size(V) << std::endl);
    //   RCLCPP_INFO_STREAM(get_logger(), "Z " << arma::size(Z) << std::endl);

    arma::vec A;

    if (s.min() < 10.0e-12) {
      A = V.col(3);
    } else {
      arma::mat Y = V * arma::diagmat(s) * V.t();

      // RCLCPP_INFO_STREAM(get_logger(), "Y " << arma::size(Y) << std::endl);

      arma::mat Q = Y * H_inv * Y;

      // RCLCPP_INFO_STREAM(get_logger(), arma::size(Q) << std::endl);

      arma::vec eigval;
      arma::mat eigvec;

      arma::eig_sym(eigval, eigvec, Q);

      size_t min_pos_index = 0;
      double min_val = 10.0e6;

      for (int i = 0; i < static_cast<int>(arma::size(eigval)(0)); i++) {
        if (eigval(i) < min_val && eigval(i) > 0.0) {
          min_val = eigval(i);
          min_pos_index = i;
        }
      }
      A = arma::solve(Y, eigvec.col(min_pos_index));
      // RCLCPP_INFO_STREAM(get_logger(), eigval <<std::endl);
      // RCLCPP_INFO_STREAM(get_logger(), "eigvec: " << eigvec <<std::endl);
    }

    double a = -A(1) / 2.0 / A(0);
    double b = -A(2) / 2.0 / A(0);
    double R_squared = (A(1) * A(1) + A(2) * A(2) - 4.0 * A(0) * A(3)) / 4.0 / A(0) / A(0);
    double R = std::sqrt(R_squared);

    double c_x = a + means(0);
    double c_y = b + means(1);
    return {c_x, c_y, R};
  }



}