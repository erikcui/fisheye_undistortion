//
// Created by yukan on 19-1-29.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

Eigen::Vector3d cam2world_pinhole(double x, double y, cv::Mat &cameraMatrix);
int main(){

    // UNDISTORTION
    cv::Mat img = cv::imread("/home/yukan/Pictures/images_190128/step7_20/08195_cam2.jpg");
    cv::Mat img_undistorted;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1932.848472028463,0.0,668.2696121566152,
                                                     0,1932.515499213407,315.4113522020265,
                                                     0.0,0.0,1.0);
    cv::Vec4d distortion(-0.466433006433524,0.904493119268293, 0 ,0);
    cv::undistort(img, img_undistorted, cameraMatrix, distortion);

    Eigen::Vector3d bearing = cam2world_pinhole(100.0, 100.0, cameraMatrix);
    std::cout << bearing << std::endl;
    return 0;
}


Eigen::Vector3d cam2world_pinhole(double x, double y, cv::Mat &cameraMatrix){
    cv::Point2d pt(x,y);
    cv::Point3d ptHom(pt.x, pt.y, 1.0);
    cv::Mat bearing_cv = cameraMatrix.inv() * cv::Mat(ptHom);
    Eigen::Vector3d bearing;
    cv::cv2eigen(bearing_cv, bearing);
    bearing.normalize();
    return bearing;

}




