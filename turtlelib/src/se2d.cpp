#include "turtlelib/se2d.hpp"

namespace turtlelib {

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw){
        os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw){
        if (is.peek() == '['){
            is.get();
        }
        is >> tw.omega >> tw.x >> tw.y;
        while (is.get() != '\n');
        return (is);
    }

    // empty initialization (identity)
    Transform2D::Transform2D(): trans{0.0, 0.0}, rot{0.0} {};

    // translation only
    Transform2D::Transform2D(Vector2D trans): trans{trans}, rot{0.0} {}; 

    // rotation only
    Transform2D::Transform2D(double radians): trans{0.0, 0.0}, rot{radians} {};

    // translation and rotation
    Transform2D::Transform2D(Vector2D trans, double radians): trans{trans}, rot{radians} {};

    // transform point
    Point2D Transform2D::operator()(Point2D p) const {
        Point2D result{};
        result.x = std::cos(rot) * p.x - std::sin(rot) * p.y + trans.x;
        result.y = std::sin(rot) * p.x + std::cos(rot) * p.y + trans.y;
        return (result);
    }

    // transform vector
    Vector2D Transform2D::operator()(Vector2D v) const {
        Vector2D result{};
        result.x = std::cos(rot) * v.x - std::sin(rot) * v.y;
        result.y = std::sin(rot) * v.x + std::cos(rot) * v.y;
        return (result);
    }

    // transform twist
    Twist2D Transform2D::operator()(Twist2D v) const {
        Twist2D result{};
        result.omega = v.omega;
        result.x = trans.y * v.omega + std::cos(rot) * v.x - std::sin(rot) * v.y;
        result.y = -trans.x * v.omega + std::sin(rot) * v.x + std::cos(rot) * v.y;
        return (result);
    }

    // get the inverse of a transform
    Transform2D Transform2D::inv() const {
        Transform2D result{};
        result.rot = -rot;
        result.trans.x = -trans.x * std::cos(rot) - trans.y * std::sin(rot);
        result.trans.y = -trans.y * std::cos(rot) + trans.x * std::sin(rot);
        return (result);
    }

    // compose transforms
    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        auto theta{rot+rhs.rotation()};
        auto x{std::cos(rot) * rhs.translation().x - std::sin(rot) * rhs.translation().y + trans.x};
        auto y{std::sin(rot) * rhs.translation().x + std::cos(rot) * rhs.translation().y + trans.y};
        rot = theta;
        trans.x = x;
        trans.y = y;
        return *this;
    }

    // return translation
    Vector2D Transform2D::translation() const {
        return (trans);
    }

    // return rotation
    double Transform2D::rotation() const {
        return (rot);
    }

    // return Transfrom in string form
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
    /// deg: 90 x: 3 y: 5
        os << "deg: " << rad2deg(tf.rot) << " x: " << tf.trans.x << " y: " << tf.trans.y;
        return (os);
    }

    // read transform in string form
    // deg: 90 x: 3 y: 5
    // 90 3 5
    // 90\n3\n5\n
    std::istream & operator>>(std::istream & is, Transform2D & tf) {
        double rot{};
        double x{};
        double y{};

        if (is.peek() == 'd'){
            while (!std::isdigit(is.peek()) && is.peek()!='-'){
                is.get();
            }
            is >> rot;
            while (!std::isdigit(is.peek()) && is.peek()!='-'){
                is.get();
            }
            is >> x;
            while (!std::isdigit(is.peek()) && is.peek()!='-'){
                is.get();
            }
            is >> y;
        } else {
            is >> rot >> x >> y;
        }
        tf = Transform2D{{x, y}, deg2rad(rot)};
        return (is);
    }

    // product of two transforms
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs*=rhs;
    }

    Transform2D integrate_twist(Twist2D twist){
        if (twist.omega == 0.0){    // if twist is pure translation
            return Transform2D{Vector2D{twist.x, twist.y}};
        } else {    // if twist includes rotation
            auto Tsb = Transform2D{Vector2D{twist.y/twist.omega, -twist.x/twist.omega}};
            auto Tbs = Tsb.inv();
            auto Tssprime = Transform2D{twist.omega};
            return Tbs * Tssprime * Tsb;
        }
    }
}