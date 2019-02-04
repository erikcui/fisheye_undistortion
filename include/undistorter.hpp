//
// Created by yukan on 19-1-7.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#ifndef FISHEYE_UNDISTORT_UNDISTORTER_HPP
#define FISHEYE_UNDISTORT_UNDISTORTER_HPP


class undistorter {

public:
    undistorter();
    cv::Mat apply_transform(cv::Mat &img_undistorted);

    Eigen::Vector3d camToWorld(Eigen::Vector2d &imgPt, Eigen::Matrix<double, 5, 1> &intrinsic,
                               Eigen::Vector4d &distortion);
    Eigen::Vector2d worldToCam(Eigen::Vector3d &bearingV, Eigen::Matrix<double, 5, 1> &intrinsic,
                               Eigen::Vector4d &distortion);
    void undistort(Eigen::Vector2d &imgPt, Eigen::Vector4d &distortion);
    void distort(Eigen::Vector2d &imgPt, Eigen::Vector4d &distortion);
    void distort(Eigen::Vector2d &imgPt, Eigen::Vector4d &distortion, Eigen::Matrix2d &J);

};


#endif //FISHEYE_UNDISTORT_UNDISTORTER_HPP
