#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib {

    DiffDrive::DiffDrive(double track, double radius){
        wheel_track = track;
        wheel_radius = radius;
        InitializePseudoInv();
    }

    void DiffDrive::InitializePseudoInv(){
        // Moore-Penrose left-pseudoinverse: ((A*A)^(-1))A*
        left_pinv.p11 = -wheel_track/wheel_radius;
        left_pinv.p12 = 1.0/wheel_radius;
        left_pinv.p13 = 0.0;
        left_pinv.p21 = wheel_track/wheel_radius;
        left_pinv.p22 = 1.0/wheel_radius;
        left_pinv.p23 = 0.0;
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
        //phiLeft = normalize_angle(rad_left);
        phi_left = rad_left;
        //phiRight = normalize_angle(rad_right);
        phi_right = rad_right;
        return config;
    }

    wheelSpeeds DiffDrive::IKin(Twist2D twist){
        if (!almost_equal(twist.y, 0.0)){
            throw std::logic_error("wheels would slip for input twist!");
        }
        auto vel_left = left_pinv.p11*twist.omega+left_pinv.p12*twist.x;
        auto vel_right = left_pinv.p21*twist.omega+left_pinv.p22*twist.x;
        return {vel_left,vel_right};
    }

    Transform2D DiffDrive::getConfig(){
        return config;
    }




}