#include <iostream>
#include <opencv2/opencv.hpp>
//#include "opencv2/imgproc.hpp"
#include "Drawer.h"
#include "CameraCalibrate.h"
#include "Shape.h"
/**
 * Modes for thr video loop
 * NORMAL: shows the corners of chessboard if detected and allows user to caliberate and shows rms error
 * POSE: shows the rvec and tvec in video in real time along with 3d axes and boundary
 * PLAY: extension mode to allow to create  3d shapes in AR and play around with them
 *
 */
enum MODE{
    NORMAL =1,
    POSE =2,
    PLAY=3
};


//below for loop has 3 modes NORMAL mode, POSE mode and PLAY mode
void video(MODE mode){
    cv::VideoCapture *capdev;
    capdev = new cv::VideoCapture(0);

    // OR advance usage: select any API backend
    int deviceID = 0;        // 0 = open default camera
    int apiID = cv::CAP_ANY; // 0 = autodetect default API
    // open selected camera using selected API
    capdev->open(deviceID, apiID);

    // check if we succeeded
    if (!capdev->isOpened())
    {
        printf("Unable to open video device\n");
        exit(-1);
    }

    // get some properties of the image
    cv::Size refS((int)capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int)capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    cv::namedWindow("Video", 1); // identifies a window
    cv::Mat frame;

    //--- GRAB AND WRITE LOOP
    std::cout << "Start grabbing" << std::endl;
    cv:: Mat tgray;
    cv::Mat cropped;
    std::vector<std::vector<cv::Point3f>> world_3d_points_list;
    std::vector<std::vector<cv::Point2f>> corners_list;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat rvec_read;
    cv::Mat tvec_read;
    char pose_result[500];
    double rms=10000;
    char key;
    Drawer* drawer;
    std::vector<Shape*> shapes_vector;
    int shape_idx=0;
    bool need_axes_help= false;
    for (;;) {
        // wait for a new frame from camera and store it into 'frame'
        bool didgrab = capdev->read(frame); // get a new frame from the camera, treat as a stream
        // check if we succeeded
        if (!didgrab) {
            printf("Unable to grab image\n");
            break;
        } else if (frame.empty()) {
            printf("frame is empty\n");
            break;
        }

        // Setup a rectangle to define your region of interest
        cv::Rect myROI(0.22*frame.cols, 0.2*frame.rows, 0.5*frame.cols, 0.65*frame.rows);// x right y below
        cv::Mat croppedRef(frame, myROI);
        croppedRef.copyTo(cropped);
        cv::cvtColor(cropped, tgray,cv::COLOR_BGR2GRAY);


        cv::Size patternsize(9,6); //interior number of corners
        std::vector<cv::Point2f> corners; //this will be filled by the detected corners
        //CALIB_CB_FAST_CHECK saves a lot of time on images
        //that do not contain any chessboard corners
        bool patternfound = findChessboardCorners(tgray, patternsize, corners,
                                                  cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                                                     + cv::CALIB_CB_FAST_CHECK);

        if(patternfound){
            cornerSubPix(tgray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001 ));
            if(mode==NORMAL)
                //show the corners visual by opencv but in normal mode not in pose mode because pose mode shows t,r on top and 3d axes
                drawChessboardCorners(cropped, patternsize, cv::Mat(corners), patternfound);
        }

        std::cout<<"frame size"<<tgray.rows<<"x"<<tgray.cols<<"\n";


        key = cv::waitKey(10);

        if (key == 'q')
        {
            break;
        }

        //till here above part is common

        if(patternfound && (mode==POSE || mode == PLAY)){
            //pose mode so try to find pose of this frame
            //get camera matrix, dist coeff
            cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
            distCoeffs = cv::Mat::zeros(8, 1, CV_64FC1);
            read_camera_results(cameraMatrix,distCoeffs);
            std::vector<cv::Point3f> world_corners;
            getWorld3dPoints(patternsize, world_corners); // gets 54 world coordinates  like 000 100 200
            //solve pnp to get rvec and tvec
            cv::solvePnP(world_corners,corners,cameraMatrix,distCoeffs,rvec_read,tvec_read);

            //CREATE THE DRAWER OBJECT
            drawer = new Drawer(cropped,cameraMatrix,distCoeffs,rvec_read,tvec_read);

            if(mode ==POSE){
                drawer->draw3d_axes();
                drawer->draw_Boundary();
            }
            else{
                //PLAY MODE
                //for all the shapes in vector redraw them for this frame
                for(auto & i : shapes_vector){
                    i->draw(drawer);
                }
            }

        }

        //NORMAL MODE KEYMAPS
        if(patternfound && mode ==NORMAL){
            if (key == 's')
            {
                //SAVING RESULTS IN NORMAL MODE
                if(corners.size()==54) {
                    corners_list.push_back(corners); //push the new corners_set of size 54
                    std::vector<cv::Point3f> world_corners;
                    getWorld3dPoints(patternsize, world_corners); // gets 54 world coordinates  like 000 100 200
                    world_3d_points_list.push_back(world_corners); //everytime user pushes s

                    cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
                    init_camera_matrix(cameraMatrix, tgray.rows, tgray.cols);
                    distCoeffs = cv::Mat::zeros(8, 1, CV_64FC1);

                    rms = cv::calibrateCamera(world_3d_points_list, corners_list, tgray.size(), cameraMatrix, distCoeffs,
                                              rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO);

                    //overwrite results to file
                    //overwrite tvec and rvec for all images and new image
                    for(int c =0;c<corners_list.size();c++){
                        save_results(c,rvecs[c],tvecs[c]);
                    }
                    //index of this latest image
                    int new_idx = corners_list.size()-1;
                    cv::imwrite("../"+std::to_string(new_idx)+".jpg", tgray);
                }
            }
            if(key=='c'){
                //overwrite camera matrix and distcoeff in camera.txt
                save_camera_results(cameraMatrix,distCoeffs);
            }
        }


        //PLAY MODE KEYMAPS
        else if(patternfound && mode==PLAY){
            float step=0.1;
            if(!shapes_vector.empty()){
                switch(key){
                    case 'X'://increase center X of shape object
                        shapes_vector[shape_idx]->x+=step;break;
                    case 'Y':
                        shapes_vector[shape_idx]->y+=step;break;
                    case 'Z':
                        shapes_vector[shape_idx]->z+=step; break;
                    case 'R'://increase radius of shape object
                        shapes_vector[shape_idx]->r+=step; break;
                    case 'H'://increase Height of shape object
                        shapes_vector[shape_idx]->h+=step; break;
                    case 'x'://decrease center x of shape object
                        shapes_vector[shape_idx]->x-=step;break;
                    case 'y':
                        shapes_vector[shape_idx]->y-=step;break;
                    case 'z':
                        if(shapes_vector[shape_idx]->z>0){
                            shapes_vector[shape_idx]->z-=step; break;
                        }
                    case 'r': //decrease radius of shape object
                        if(shapes_vector[shape_idx]->r>0){
                            shapes_vector[shape_idx]->r-=step; break;
                        }
                    case 'h': //decrease height of shape object
                        if(shapes_vector[shape_idx]->h>0){
                            shapes_vector[shape_idx]->h-=step; break;
                        }
                    case 't': //shift between shapes
                        shapes_vector[shape_idx]->marked= false;
                        shape_idx=(shape_idx+1)%(shapes_vector.size());
                        shapes_vector[shape_idx]->marked= true;break;

                    case 'T'://also shift between shapes made in caps too because it will be used more frequently
                        shapes_vector[shape_idx]->marked= false;
                        shape_idx=(shape_idx+1)%(shapes_vector.size());
                        shapes_vector[shape_idx]->marked= true;break;
                    case 'd': // deletes the current selected shape object
                        shapes_vector.erase(shapes_vector.begin()+shape_idx);break;
                    case 'S': //saves the current frame
                        cv::imwrite("../saved.jpg", cropped);
                    case 'p': //shifts between the planes XY, YZ AND XZ
                        int curr_p=shapes_vector[shape_idx]->p;
                        shapes_vector[shape_idx]->p = static_cast<PLANE>((curr_p+1)%3); break;


                }

            }

            //creating shapes
            switch(key){
                case 's': //creates sphere
                    shapes_vector.push_back(new Sphere(0, 0, 0, 1)); break;
                case 'c'://creates cylinder
                    shapes_vector.push_back(new Cylinder(0,0, 1, 1, 4)); break;
                case 'o'://creates right angle cone
                    shapes_vector.push_back(new Cone(0, 0, 1, 1, 4)); break;
                case 'u'://creates cube
                    shapes_vector.push_back(new Cube( 2, 2, 1, 1)); break;
                case 'v'://if user is lost about axes use this to see axes and press again to make it go away
                    need_axes_help=!need_axes_help;
            }

            //3d axes helper
            if(need_axes_help){
                drawer->draw3d_axes();
            }

        }

        //below part is which image to display and what is title
        if(patternfound){
            if(mode==POSE){
                sprintf(pose_result, "R = (%lf, %lf, %lf), T = (%lf, %lf, %lf)",
                        rvec_read.at<double>(0,0),rvec_read.at<double>(1,0),
                        rvec_read.at<double>(2,0),tvec_read.at<double>(0,0),
                        tvec_read.at<double>(1,0),tvec_read.at<double>(2,0));
                cv::setWindowTitle("video",pose_result);
            }
            else if(mode==NORMAL){
                cv::setWindowTitle("video"," corners=: "+std::to_string(corners.size()) +" total calibration frames:"+std::to_string(corners_list.size())+"  current_rms_error:"+std::to_string(rms));
            }
            else if(mode ==PLAY){
                cv::setWindowTitle("video","Have Fun!");
            }
            cv::imshow("video",cropped); //drawn with corners by opencv

        }else{
            cv::setWindowTitle("video","No pattern found");
            cv::imshow("video",tgray);
        }

    }

    delete capdev;

}

/**
 * Implements Last task about detecting features out of other patterns
 */
void HarrisCorner() {
    cv::VideoCapture *capdev;
    capdev = new cv::VideoCapture(0);

    // OR advance usage: select any API backend
    int deviceID = 0;        // 0 = open default camera
    int apiID = cv::CAP_ANY; // 0 = autodetect default API
    // open selected camera using selected API
    capdev->open(deviceID, apiID);

    // check if we succeeded
    if (!capdev->isOpened()) {
        printf("Unable to open video device\n");
        exit(-1);
    }

    // get some properties of the image
    cv::Size refS((int) capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    cv::namedWindow("Video", 1); // identifies a window
    cv::Mat frame;

    //--- GRAB AND WRITE LOOP
    std::cout << "Start grabbing" << std::endl;
    cv::Mat tgray;
    cv::Mat cropped;

    for (;;) {
        // wait for a new frame from camera and store it into 'frame'
        bool didgrab = capdev->read(frame); // get a new frame from the camera, treat as a stream
        // check if we succeeded
        if (!didgrab) {
            printf("Unable to grab image\n");
            break;
        } else if (frame.empty()) {
            printf("frame is empty\n");
            break;
        }

        // Setup a rectangle to define your region of interest
        cv::Rect myROI(0.22 * frame.cols, 0.2 * frame.rows, 0.5 * frame.cols, 0.65 * frame.rows);// x right y below
        cv::Mat croppedRef(frame, myROI);
        croppedRef.copyTo(cropped);
        cv::cvtColor(cropped, tgray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(tgray,corners,100,0.1,10);

        for (int idx = 0; idx < corners.size(); idx++) {
            cv::circle(cropped,corners.at(idx),2,cv::Scalar(0,0,255),2);
        }

        cv::setWindowTitle("video"," corners=: "+std::to_string(corners.size()));


        char key = cv::waitKey(10);

        if (key == 'q')
        {
            break;
        }

        cv::imshow("video",cropped); //drawn with corners by opencv

    }
}
int main(int argc, char *argv[]) {
    if(argc>1){
        if(strcmp("pose", argv[1])==0){
            video(POSE);
        }
        else if(strcmp("play", argv[1])==0) {
            video(PLAY);
        }
        else if(strcmp("harris",argv[1])==0){
            HarrisCorner();
        }
    }else{
        video(NORMAL);
    }

}







