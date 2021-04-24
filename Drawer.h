//
// Created by Ishan on 3/15/21.
//

#ifndef PROJ4_DRAWER_H
#define PROJ4_DRAWER_H
#include <opencv2/opencv.hpp>

enum PLANE
{   XY = 0,
    YZ = 1,
    XZ = 2,
};
/**
 * This class is used to draw 3d shapes on the image using AR
 */
class Drawer {
private:

    /**
     * Helper function to draw square in any plane given the x,y,z
     * @param imagePoints the 2d points to map the 3d points to
     * @param side
     * @param x
     * @param y
     * @param z
     * @param p
     */
    void drawSquare(std::vector<cv::Point2f> &imagePoints, float side, float x, float y, float z,PLANE p);


    /**
     * Simply draws a circle and puts the 2d points in imagePoints
     * @param cameraMatrix
     * @param distCoeffs
     * @param rvec_read
     * @param tvec_read
     * @param imagePoints
     * @param r
     * @param x center x
     * @param y center y
     * @param z center z
     * @param p PLANE for chopped circles
     */
    void drawCircle(std::vector<cv::Point2f> &imagePoints, float r, float x, float y, float z,PLANE p);

    /**
     * helper for drawOvals
     * @param radius_factor value from 0 to 1
     * @param imagePoints
     * @param x
     * @param y
     * @param z
     */
    void drawOval(float radius_factor, std::vector<cv::Point2f> &imagePoints, float x, float y, float z);

public:
    cv::Mat image;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat rvec_read;
    cv::Mat tvec_read;

    /**
     * creates drawer object using this parameters below which are enough to project 3d to 2d
     * @param image the image
     * @param cameraMatrix the camera matrix
     * @param distCoeffs the distortion coefficients
     * @param rvec_read the rvec of current position of camera
     * @param tvec_read the tvec of current position of camera
     */
    Drawer(cv::Mat &image, cv::Mat &cameraMatrix,cv::Mat &distCoeffs,cv::Mat &rvec_read,cv::Mat &tvec_read);

    /**
     * Just projects the 3d axes into 2d
     */
    void draw3d_axes();


    /**
     * Just draws boundary in the 3d and projects into 2d
     */
    void draw_Boundary();

    /**
     * draws a cube in the 3d and projects into 2d
     * @param x the corner of cube x,y,z
     * @param y
     * @param z
     * @param side
     * @param color
     * @param pl the plane of sides because the cube is 4 faced
     */
    void drawCube(float x, float y,float z, float side,cv::Scalar &color,PLANE pl);

    /**
     * draws a cylinder in the 3d and projects into 2d
     *
     * the center of base of cylinder x,y,z
     * @param x
     * @param y
     * @param z
     * @param radius
     * @param h height
     * @param p
     * @param color
     */
    void drawCylinder(float x, float y,float z, float radius,float h, PLANE p,cv::Scalar &color);

    /**
     * draws a cylinder in the 3d and projects into 2d
     *
     * center of base x,y,z
     * @param x
     * @param y
     * @param z
     * @param radius
     * @param h
     * @param p
     * @param color
     */
    void drawCone(float x, float y,float z, float radius,float h, PLANE p,cv::Scalar &color);

    /**
     * draws a sphere in 3d and projects into 2d (this one was tough specially thhe relation between radius of discs and their height
     * @param x center of sphere x,y,z
     * @param y
     * @param z
     * @param radius
     * @param color
     */
    void drawSphere(float x, float y, float z, float radius,cv::Scalar &color);

    /**
    * Draws oval shapes. doesnt look so nice and so i am not using it in PLAY mode so you can skip it
    */
    void drawOvals();
};


#endif //PROJ4_DRAWER_H
