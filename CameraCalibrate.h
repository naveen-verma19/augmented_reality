//
// Created by Naveen and Ishan on 3/15/21.
//

#include <opencv2/opencv.hpp>


#ifndef PROJ4_CAMERACALIBRATE_H
#define PROJ4_CAMERACALIBRATE_H
/**
 *Gets the 3d coordinates of the world corresponding to the corners got from getChessboard corners function(row major)
 * @param boardSize
 * @param corners
 */
void getWorld3dPoints(cv::Size &boardSize, std::vector<cv::Point3f>& corners);

/**
 * Initiallises camera matrix to a theoretically correct fa and fc
 */
void init_camera_matrix( cv::Mat &cameraMatrix,int rows,int cols);


/**
 * Saves matrix into vector for saving to file
 */
void convert_mat_to_vector(cv::Mat &cameraMatrix, std::vector<double> &vec);

/**
 * Saves the rotation and traslation vectors to file
 * @param idx the index of caliberation
 * @param rvec the rotation vector 3X1
 * @param tvec the translation vector 3X1
 */
void save_results(int idx,cv::Mat &rvec, cv::Mat &tvec);

/**
 * Saves the camera results to camera.txt
 * @param cameraMatrix the 3x3 camera matrix
 * @param distCoeffs the 5 distortion coefficients
 */
void save_camera_results(cv::Mat &cameraMatrix,cv::Mat &distCoeffs);


/**
 * Reads camera results into the below fields
 * @param cameraMatrix
 * @param distCoeffs
 */
void read_camera_results(cv::Mat &cameraMatrix,cv::Mat &distCoeffs);
#endif //PROJ4_CAMERACALIBRATE_H
