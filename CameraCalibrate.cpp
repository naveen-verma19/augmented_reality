//
// Created by Naveen and Ishan on 3/15/21.
//

#include "CameraCalibrate.h"
#include <opencv2/opencv.hpp>
#include <fstream>

void getWorld3dPoints(cv::Size &boardSize, std::vector<cv::Point3f>& corners)
{
    corners.clear();
    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            corners.push_back(cv::Point3f(j, -i, 0));

}


void init_camera_matrix( cv::Mat &cameraMatrix,int rows,int cols){
    cameraMatrix.at<double>(0,1)=cameraMatrix.at<float>(1,0)=cameraMatrix.at<float>(2,0)=cameraMatrix.at<float>(2,1)=0;
    cameraMatrix.at<double>(0,2)=float(cols)/2;
    cameraMatrix.at<double>(1,2)=float(rows)/2;
}


void convert_mat_to_vector(cv::Mat &cameraMatrix, std::vector<double> &vec){
    vec.clear();
    for(int i=0;i<cameraMatrix.rows;i++){
        for(int j=0;j<cameraMatrix.cols;j++){
            vec.push_back(cameraMatrix.at<double>(i,j));
        }
    }
}


void save_results(int idx,cv::Mat &rvec, cv::Mat &tvec){

    std::string f="../"+std::to_string(idx)+".txt";
    std::ofstream outfile(f);//overwrite the file

    //files are stored with rvec followed by tvec
    std::vector<double> results;
    convert_mat_to_vector(rvec,results);
    for(double val:results){
        outfile<<val<<"\n";
    }
    convert_mat_to_vector(tvec,results);
    for(double val:results){
        outfile<<val<<"\n";
    }
    outfile.close();


}

void save_camera_results(cv::Mat &cameraMatrix,cv::Mat &distCoeffs){
    std::string f="../camera.txt";
    std::vector<double> results;
    std::ofstream outfile2(f);//overwrite the file
    convert_mat_to_vector(cameraMatrix,results);
    for(double val:results){
        outfile2<<val<<"\n";
    }

    convert_mat_to_vector(distCoeffs,results);
    for(double val:results){
        outfile2<<val<<"\n";
    }

    outfile2.close();
}


void read_camera_results(cv::Mat &cameraMatrix,cv::Mat &distCoeffs){
    std::string f="../camera.txt";
    std::vector<double> results;
    std::ifstream infile(f);//read file
    double val;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            infile>>val;
            cameraMatrix.at<double>(i,j)=val;
        }
    }

    for(int i=0;i<5;i++){
        infile>>val;
        distCoeffs.at<double>(i,0)=val;
    }

    infile.close();
}