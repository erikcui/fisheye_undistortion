//
// Created by yukan on 19-1-7.
//

#include "../include/undistorter.hpp"


undistorter::undistorter() {}


cv::Mat undistorter::apply_transform(cv::Mat &img_undistorted) {
    for (int i = 0; i < img_undistorted.rows; ++i) {
        for (int j = 0; j < ; ++j) {
            
        }
    }

}


Eigen::Vector3d
undistorter::camToWorld(Eigen::Vector2d &imgPt, Eigen::Matrix<double, 5, 1> &intrinsic,
                           Eigen::Vector4d &distortion){
    double xi = intrinsic[0];
    double fu = intrinsic[1];
    double fv = intrinsic[2];
    double cu = intrinsic[3];
    double cv = intrinsic[4];

    double k1 = distortion[0];
    double k2 = distortion[1];
    double p1 = distortion[2];
    double p2 = distortion[3];

    Eigen::Vector3d outPoint;

    // updateTemporaries()

    double recip_fu = 1.0 / fu;
    double recip_fv = 1.0 / fv;
    double fu_over_fv = fu / fv;
    double one_over_xixi_m_1 = 1.0 / (xi * xi - 1.0);
    double fov_parameter = (xi <= 1.0)? xi : 1.0 / xi;

    //Unproject
    outPoint[0] = recip_fu * (imgPt[0] - cu);
    outPoint[1] = recip_fv * (imgPt[1] - cv);

    //Re-distort

    Eigen::Vector2d temp = outPoint.block<2,1>(0,0);
    undistort(temp, distortion);
    outPoint.block<2,1>(0,0) = temp;

    double rho2_d = outPoint[0] * outPoint[0] + outPoint[1] * outPoint[1];
    outPoint[2] = 1.0 - xi * (rho2_d + 1.0) / (xi + sqrt(1.0 + (1.0 - xi * xi) * rho2_d));
    return outPoint;

}

Eigen::Vector2d
undistorter::worldToCam(Eigen::Vector3d &bearingV, Eigen::Matrix<double, 5, 1> &intrinsic,
                           Eigen::Vector4d &distortion){
    Eigen::Vector2d outKeypoint;
    double d = bearingV.norm();

    double xi = intrinsic[0];
    double fu = intrinsic[1];
    double fv = intrinsic[2];
    double cu = intrinsic[3];
    double cv = intrinsic[4];

    double k1 = distortion[0];
    double k2 = distortion[1];
    double p1 = distortion[2];
    double p2 = distortion[3];

    Eigen::Vector3d outPoint;

    // updateTemporaries()

    double recip_fu = 1.0 / fu;
    double recip_fv = 1.0 / fv;
    double fu_over_fv = fu / fv;
    double one_over_xixi_m_1 = 1.0 / (xi * xi - 1.0);
    double fov_parameter = (xi <= 1.0)? xi : 1.0 / xi;

    // check if point will lead to a valid projection
    if(bearingV[2] <= -(fov_parameter * d))
        return outKeypoint;

    double rz = 1.0 / (bearingV[2] + xi * d);
    outKeypoint[0] = bearingV[0] * rz;
    outKeypoint[1] = bearingV[1] * rz;

    distort(outKeypoint, distortion);

    outKeypoint[0] = fu * outKeypoint[0] + cu;
    outKeypoint[1] = fv * outKeypoint[1] + cv;


}


void
undistorter::undistort(Eigen::Vector2d &imgPt, Eigen::Vector4d &distortion){
    Eigen::Vector2d ybar = imgPt;
    const int n = 5;
    Eigen::Matrix2d F;
    Eigen::Vector2d y_tmp;
    for (int i = 0; i < n; i++) {
        y_tmp = ybar;
        distort(y_tmp, distortion, F);

        Eigen::Vector2d e(imgPt - y_tmp);
        Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;
        ybar += du;
        if(e.dot(e) < 1e-15)
            break;

    }

    imgPt = ybar;
}

void
undistorter::distort(Eigen::Vector2d &imgPt, Eigen::Vector4d &distortion){
    double k1 = distortion[0];
    double k2 = distortion[1];
    double p1 = distortion[2];
    double p2 = distortion[3];

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = imgPt[0] * imgPt[0];
    my2_u = imgPt[1] * imgPt[1];
    mxy_u = imgPt[0] * imgPt[1];
    rho2_u = mx2_u + my2_u;

    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

    imgPt[0] += imgPt[0] * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u);
    imgPt[1] += imgPt[1] * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);

}

void
undistorter::distort(Eigen::Vector2d &imgPt, Eigen::Vector4d &distortion, Eigen::Matrix2d &J){
    double k1 = distortion[0];
    double k2 = distortion[1];
    double p1 = distortion[2];
    double p2 = distortion[3];

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;
    J.setZero();
    mx2_u = imgPt[0] * imgPt[0];
    my2_u = imgPt[1] * imgPt[1];
    mxy_u = imgPt[0] * imgPt[1];
    rho2_u = mx2_u + my2_u;

    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

    J(0,0) = 1.0 + rad_dist_u + k1 * 2.0 * mx2_u + k2 * rho2_u * 4.0 * mx2_u
             + 2.0 * p1 * imgPt[1] + 6.0 * p2 * imgPt[0];
    J(1,0) = k1 * 2.0 * imgPt[0] * imgPt[1] + k2 * 4.0 * rho2_u * imgPt[0] * imgPt[1]
             + p1 * 2.0 * imgPt[0] + 2.0 * p2 *  imgPt[1];
    J(0,1) = J(1,0);
    J(1,1) = 1.0 + rad_dist_u + k1 * 2.0 * my2_u + k2 * rho2_u * 4.0 * my2_u
             + 6.0 * p1 * imgPt[1] + 2.0 * p2 * imgPt[0];

    imgPt[0] += imgPt[0] * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u);
    imgPt[1] += imgPt[1] * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}