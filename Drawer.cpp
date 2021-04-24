//
// Created by Naveen Verma on 3/15/21.
//

#include "Drawer.h"

/**
 * This implements the above interface please read that for documentation
 */

Drawer::Drawer(cv::Mat &image, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &rvec_read, cv::Mat &tvec_read) {
    this->image=image;
    this->cameraMatrix=cameraMatrix;
    this->distCoeffs=distCoeffs;
    this->rvec_read=rvec_read;
    this->tvec_read=tvec_read;
}


void Drawer::draw3d_axes() {
    std::vector<cv::Point3f> world_boundary;
    world_boundary.push_back(cv::Point3f(0, 0, 0));
    world_boundary.push_back(cv::Point3f(0, 0, 2));
    world_boundary.push_back(cv::Point3f(0, 2, 0));
    world_boundary.push_back(cv::Point3f(2, 0, 0));
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(world_boundary,rvec_read,tvec_read,cameraMatrix,distCoeffs,imagePoints); // 3d points converted to 2d image plane

    for(const auto& x: imagePoints){
        cv::circle(image,x,2,cv::Scalar(0,255,0),2);
    }
    cv::line(image,imagePoints[0],imagePoints[1],cv::Scalar(0,0,255),2);
    cv::line(image,imagePoints[0],imagePoints[2],cv::Scalar(255,0,0),2);
    cv::line(image,imagePoints[0],imagePoints[3],cv::Scalar(0,255,255),2);
}

void Drawer::draw_Boundary() {
    std::vector<cv::Point3f> world_boundary;
    world_boundary.push_back(cv::Point3f(-1, 1, 0));
    world_boundary.push_back(cv::Point3f(-1, -6, 0));
    world_boundary.push_back(cv::Point3f(9, -6, 0));
    world_boundary.push_back(cv::Point3f(9, 1, 0));

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(world_boundary,rvec_read,tvec_read,cameraMatrix,distCoeffs,imagePoints); // 3d points converted to 2d image plane

    for(const auto& x: imagePoints){
        cv::circle(image,x,2,cv::Scalar(0,255,0),2);
    }
    for(int i=0;i<4;i++){
        cv::line(image,imagePoints[i],imagePoints[(i+1)%4],cv::Scalar(255,0,255),2);
    }

}

void Drawer::drawCube(float x, float y, float z, float side, cv::Scalar &color, PLANE pl) {
    std::vector<cv::Point2f> imagePoints;

    for(float k=0;k<side;k+=0.05){
        if(pl==XY)
            drawSquare(imagePoints,side,x,y,k+z,pl);
        if(pl==YZ)
            drawSquare(imagePoints,side,k+x,y,z,pl);
        else if(pl==XZ)
            drawSquare(imagePoints,side,x,k+y,z,pl);
        int n =imagePoints.size();
        for(const auto& v: imagePoints){
            cv::circle(image,v,2,cv::Scalar(0,255,0),1);
        }
        for(int i=0;i<n;i++){
            cv::line(image,imagePoints[i],imagePoints[(i+1)%n],color,1);
        }

    }
}


void Drawer::drawSquare(std::vector<cv::Point2f> &imagePoints, float side, float x, float y, float z, PLANE p) {
    std::vector<cv::Point3f> world_boundary;
    //clockwise points
    if(p==XY){
        world_boundary.push_back(cv::Point3f(x, y, z));
        world_boundary.push_back(cv::Point3f(x+side, y, z));
        world_boundary.push_back(cv::Point3f(x+side, y+side, z));
        world_boundary.push_back(cv::Point3f(x, y+side, z));
    }
    else if(p==YZ){
        world_boundary.push_back(cv::Point3f(x, y, z));
        world_boundary.push_back(cv::Point3f(x, y+side, z));
        world_boundary.push_back(cv::Point3f(x, y+side, z+side));
        world_boundary.push_back(cv::Point3f(x, y, z+side));
    }
    else{
        world_boundary.push_back(cv::Point3f(x, y, z));
        world_boundary.push_back(cv::Point3f(x+side, y, z));
        world_boundary.push_back(cv::Point3f(x+side, y, z+side));
        world_boundary.push_back(cv::Point3f(x, y, z+side));
    }

    cv::projectPoints(world_boundary,rvec_read,tvec_read,cameraMatrix,distCoeffs,imagePoints); // 3d points converted to 2d image plane
}


void Drawer::drawCylinder(float x, float y, float z, float radius, float h, PLANE p, cv::Scalar &color) {
    std::vector<cv::Point2f> imagePoints;

    for(float k=0;k<h;k+=0.05){
        if(p==XY)
            drawCircle(imagePoints,radius,x,y,k+z,p);
        if(p==YZ)
            drawCircle(imagePoints,radius,k+x,y,z,p);
        if(p==XZ)
            drawCircle(imagePoints,radius,x,k+y,z,p);
        for(const auto& x: imagePoints){
            cv::circle(image,x,1,cv::Scalar(0,255,0),1);
        }
        int n =imagePoints.size();
        for(int i=0;i<n;i++){
            cv::line(image,imagePoints[i],imagePoints[(i+1)%n],color,1);
        }
    }
}


void Drawer::drawCircle(std::vector<cv::Point2f> &imagePoints, float r, float x, float y, float z, PLANE p) {
    std::vector<cv::Point3f> world_boundary;
    float n=40;
    double theta=2*M_PI/float(n);
    if(p==XY){
        for(int i=0;i<n;i++){
            world_boundary.push_back(cv::Point3f( x+r*cos(float(i*theta)), y+r*sin(float(i*theta)), z));
        }
    } else if(p==YZ){
        for(int i=0;i<n;i++){
            world_boundary.push_back(cv::Point3f( x,y+r*cos(float(i*theta)), z+r*sin(float(i*theta))));
        }
    }else{
        for(int i=0;i<n;i++){
            world_boundary.push_back(cv::Point3f( x+r*cos(float(i*theta)),y,z+r*sin(float(i*theta))));
        }
    }

    cv::projectPoints(world_boundary,rvec_read,tvec_read,cameraMatrix,distCoeffs,imagePoints); // 3d points converted to 2d image plane
}



void Drawer::drawCone(float x, float y, float z, float radius, float h, PLANE p, cv::Scalar &color) {
    std::vector<cv::Point2f> imagePoints;
    for(float k=0;k<h;k+=0.05){
        if(p==XY)
            drawCircle(imagePoints,radius*(1-k/h),x,y,z+k,p);
        if(p==YZ)
            drawCircle(imagePoints,radius*(1-k/h),k+x,y,z,p);
        if(p==XZ)
            drawCircle(imagePoints,radius*(1-k/h),x,k+y,z,p);
        for(const auto& x: imagePoints){
            cv::circle(image,x,1,cv::Scalar(0,255,0),1);
        }
        int n =imagePoints.size();
        for(int i=0;i<n;i++){
            cv::line(image,imagePoints[i],imagePoints[(i+1)%n],color,1);
        }
    }
}



void Drawer::drawSphere(float x, float y, float z, float radius, cv::Scalar &color) {
    std::vector<cv::Point2f> imagePoints;
    float n=400;
    double theta=2*M_PI/float(n);
    float up = z-radius;
    std::vector<std::pair<float,float>> z_r; // the relation is z = r- r*cos(theta) where theta is 0 to 2pi
    //z and r for all discs assuming z=r
    for(int i=0;i<n;i++){
        z_r.push_back(std::make_pair(radius-radius*cos(float(i*theta)),radius*sin(float(i*theta))));
    }

    for(auto p: z_r){
        drawCircle(imagePoints,p.second,x,y,p.first+up,XY);
        for(const auto& x: imagePoints){
            cv::circle(image,x,1,cv::Scalar(0,255,0),1);
        }
        int n =imagePoints.size();
        for(int i=0;i<n;i++){
            cv::line(image,imagePoints[i],imagePoints[(i+1)%n],color,1);
        }
    }
}


void Drawer::drawOvals() {
    std::vector<cv::Point2f> imagePoints_c1;
    std::vector<cv::Point2f> imagePoints_c2;
    std::vector<float> scale= {1,0.8};
    for(int k=0;k<1;k++){
        drawOval(scale[k],imagePoints_c1,0,-2.5,0);
        for(const auto& x: imagePoints_c1){
            cv::circle(image,x,2,cv::Scalar(0,255,0),2);
        }
        for(int i=0;i<16;i++){
            cv::line(image,imagePoints_c1[i],imagePoints_c1[(i+1)%16],cv::Scalar(0,0,255),2);
        }

        //drawing second circle

        drawOval(scale[k],imagePoints_c2,0,-2.5,2);
        for(const auto& x: imagePoints_c2){
            cv::circle(image,x,2,cv::Scalar(0,255,0),2);
        }
        for(int i=0;i<16;i++){
            cv::line(image,imagePoints_c2[i],imagePoints_c2[(i+1)%16],cv::Scalar(0,0,255),2);
        }
    }
    //connecting both
    for(int i=0;i<16;i++){
        cv::line(image,imagePoints_c1[i],imagePoints_c2[i],cv::Scalar(0,0,255),2);
    }
}

void Drawer::drawOval(float radius_factor, std::vector<cv::Point2f> &imagePoints, float x, float y, float z) {
    std::vector<cv::Point3f> world_boundary;
    //clockwise points
    world_boundary.push_back(cv::Point3f(0, 0.5, 0));
    world_boundary.push_back(cv::Point3f(0.25, 1, 0));
    world_boundary.push_back(cv::Point3f(1, 1.5, 0));
    world_boundary.push_back(cv::Point3f(2, 1.85, 0));
    world_boundary.push_back(cv::Point3f(3, 2, 0));
    world_boundary.push_back(cv::Point3f(4, 1.85, 0));
    world_boundary.push_back(cv::Point3f(5, 1.5, 0));
    world_boundary.push_back(cv::Point3f(5.75, 1, 0));
    world_boundary.push_back(cv::Point3f(6, 0.5, 0));


    for(auto &p: world_boundary){
        p.y = p.y-0.5;
    }

    for(int i=7;i>0;i--){
        world_boundary.push_back(cv::Point3f(world_boundary[i].x,-world_boundary[i].y,world_boundary[i].z));
    }
    //origin at left node of circle

    for(auto &p: world_boundary){
        p.x= (p.x+x)*radius_factor;
        p.y = (p.y+y)*radius_factor;
        p.z+=z;
    }

    cv::projectPoints(world_boundary,rvec_read,tvec_read,cameraMatrix,distCoeffs,imagePoints); // 3d points converted
}

