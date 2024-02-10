#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib {

    DiffDrive::DiffDrive(double track, double radius){
        wheel_track = track;
        wheel_radius = radius;
    }

    Transform2D DiffDrive::FKin(double rad_left, double rad_right){
        // get angle differences
        auto angle_diff_left = rad_left - phi_left;
        auto angle_diff_right = rad_right - phi_right;

        // body twist: Lynch, Park - Modern Robotics 13.4
        auto bodyTwist = Twist2D{wheel_radius/wheel_track*(-angle_diff_left+angle_diff_right), 
                                 wheel_radius*(angle_diff_left+angle_diff_right), 0.0};
        // integrate body twist to get Tbb'
        auto tf = integrate_twist(bodyTwist);
        // update config Twb*Tbb' = Twb'
        config *= tf;
        // track wheel angles
        phi_left = rad_left;
        phi_right = rad_right;

        return config;
    }

    wheelSpeeds DiffDrive::IKin(Twist2D twist){
        if (!almost_equal(twist.y, 0.0)){
            throw std::logic_error("wheels would slip for input twist!");
        }
        auto vel_left = -wheel_track/2.0/wheel_radius*twist.omega+1.0/wheel_radius*twist.x;
        auto vel_right = wheel_track/2.0/wheel_radius*twist.omega+1.0/wheel_radius*twist.x;
        return {vel_left,vel_right};
    }

    Transform2D DiffDrive::getConfig(){
        return config;
    }




}