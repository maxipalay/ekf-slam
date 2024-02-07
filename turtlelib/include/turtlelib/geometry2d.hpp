#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives.


#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <iostream>

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (d1-d2 < 0){
            return d1-d2 > -epsilon;
        } else {
            return d1-d2 < epsilon;
        }
        return fabs(d1-d2) < epsilon;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return (deg*PI/180.0);
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return (rad/PI*180.0);
    }

    /// \brief wrap an angle to (-PI, PI]
    /// \param rad (angle in radians)
    /// \return an angle equivalent to rad but in the range (-PI, PI]
    double normalize_angle(double rad);

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(-1e-13, 0.0), "small number almost_equal failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief a 2-Dimensional Point
    struct Point2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
    };

    /// \brief output a 2 dimensional point as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param p - the point to print
    std::ostream & operator<<(std::ostream & os, const Point2D & p);

    /// \brief input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param p [out] - output vector
    std::istream & operator>>(std::istream & is, Point2D & p);

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief add vectors, modifying the lhs
        /// \param rhs - Vector2D
        /// \return a reference to the modified vector
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief subtract vectors, modifying the lhs
        /// \param rhs - Vector2D
        /// \return a reference to the modified vector
        Vector2D & operator-=(const Vector2D & rhs);

        /// \brief multiply vector by scalar, modifying the lhs
        /// \param rhs - Vector2D
        /// \return a reference to the modified vector
        Vector2D & operator*=(const double & scalar);

    };

    /// \brief normalize a vector
    /// \param vec - the vector to normalize
    /// \return a new vector, normalized
    Vector2D normalize(const Vector2D & vec);

    /// \brief Subtracting one point from another yields a vector
    /// \param head point corresponding to the head of the vector
    /// \param tail point corresponding to the tail of the vector
    /// \return a vector that points from p1 to p2
    /// NOTE: this is not implemented in terms of -= because subtracting two Point2D yields a Vector2D
    Vector2D operator-(const Point2D & head, const Point2D & tail);

    /// \brief Adding a vector to a point yields a new point displaced by the vector
    /// \param tail The origin of the vector's tail
    /// \param disp The displacement vector
    /// \return the point reached by displacing by disp from tail
    /// NOTE: this is not implemented in terms of += because of the different types
    Point2D operator+(const Point2D & tail, const Vector2D & disp);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief add vectors, returning a new vector
    /// \param rhs - Vector2D
    /// \return a reference to the modified vector
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief subtract vectors, returning a new vector
    /// \param rhs - Vector2D
    /// \return a reference to the modified vector
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief multiply vector by scalar, returning a new vector
    /// \param lhs - Vector2D
    /// \param scalar - double scalar
    /// \return a reference to the modified vector
    Vector2D operator*(Vector2D lhs, const double & scalar);

    /// \brief multiply vector by scalar, returning a new vector
    /// \param scalar - double scalar
    /// \param rhs - Vector2D
    /// \return a reference to the modified vector
    Vector2D operator*(const double & scalar, Vector2D rhs);

    /// \brief dot product of two vectors
    /// \param lhs - Vector2D
    /// \param rhs - Vector2D
    /// \return dot product - double
    double dot(const Vector2D & lhs, const Vector2D & rhs);

    /// \brief get the magnitude of a vector
    /// \param vec - Vector2D
    /// \return magnitude - double
    double magnitude(Vector2D vec);

    /// \brief return the angle between two vectors
    /// \param vec1 - Vector2D
    /// \param vec2 - Vector2D
    /// \return the angle between the vectors (rad) - double
    double angle(const Vector2D & vec1, const Vector2D & vec2);

    /// \brief return the difference in radian between two angles, preserving sign
    /// \param before - first angle
    /// \param vec2 - second angle
    /// \return the angle between the after and before, preserving sign
    double angle_diff(const double before, const double after);
}

#endif
