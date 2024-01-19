#include "turtlelib/geometry2d.hpp"

namespace turtlelib {
    
    double normalize_angle(double rad){
        // shifting the input +pi and taking the 2pi modulus
        auto diff = std::fmod(rad+PI, 2*PI);
        // if we're on the negatives or zero, return, shifting pi
        if (diff <= 0.0){
            return (diff+PI);
        }
        // if we're in the range (0, 2pi], re-shift -pi and return
        return (diff - PI);
    }

    std::ostream & operator<<(std::ostream & os, const Point2D & p){
        os << "[" << p.x << " " << p.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Point2D & p){
        if (is.peek() == '['){
            is.get();
        }
        is >> p.x >> p.y;
        while (is.get() != '\n');
        return (is);
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail){
        return Vector2D{head.x - tail.x, head.y - tail.y};
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        return Point2D{tail.x + disp.x, tail.y + disp.y};
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]";
        return (os);
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        if (is.peek() == '['){
            is.get();
        }
        is >> v.x >> v.y;
        while (is.get() != '\n'); // are we sure we'll always get a \n ?
        return (is);
    }

    Vector2D normalize(const Vector2D & vec){
        Vector2D res;
        double mod = std::sqrt(std::pow(vec.x,2) + std::pow(vec.y,2));
        res.x = vec.x / mod;
        res.y = vec.y / mod;
        return res;
    }

}
