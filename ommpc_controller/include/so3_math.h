#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

class SO3 {
public:
    static Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
        Eigen::Matrix3d w_hat;
        w_hat << 0, -w(2), w(1),
                 w(2), 0, -w(0),
                 -w(1), w(0), 0;
        return w_hat;
    }

    static Eigen::Vector3d vee(const Eigen::Matrix3d& w_hat) {
        return Eigen::Vector3d(w_hat(2,1), w_hat(0,2), w_hat(1,0));
    }

    static Eigen::Matrix3d exp(const Eigen::Vector3d& w) {
        double theta = w.norm();
        
        if (theta < 1e-10) {
            Eigen::Matrix3d w_hat = hat(w);
            return Eigen::Matrix3d::Identity() + w_hat;
        }
        
        Eigen::Vector3d axis = w / theta;
        Eigen::Matrix3d axis_hat = hat(axis);
        
        return Eigen::Matrix3d::Identity() 
             + sin(theta) * axis_hat
             + (1 - cos(theta)) * axis_hat * axis_hat;
    }

    static Eigen::Vector3d log(const Eigen::Matrix3d& R) {
        double theta = acos((R.trace() - 1) / 2);
        
        if (theta < 1e-10) {
            return vee(R - Eigen::Matrix3d::Identity());
        }
        
        return vee((theta / (2 * sin(theta))) * (R - R.transpose()));
    }

    static Eigen::Matrix3d leftJacobian(const Eigen::Vector3d& w) {
        double theta = w.norm();
        
        if (theta < 1e-10) {
            Eigen::Matrix3d w_hat = hat(w);
            return Eigen::Matrix3d::Identity() 
                 + 0.5 * w_hat 
                 + (1.0/6.0) * w_hat * w_hat;
        }
        
        Eigen::Vector3d a = w / theta;
        Eigen::Matrix3d a_hat = hat(a);
        
        Eigen::Matrix3d J = Eigen::Matrix3d::Identity()
                          + ((1 - cos(theta)) / theta) * a_hat
                          + (1 - sin(theta) / theta) * a_hat * a_hat;
        
        return J;
    }
};
