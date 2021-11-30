#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <iostream>

#define CAMERA_ID_MAIN 0

#define CAMERA_SAMPLE_WIDTH 1280
#define CAMERA_SAMPLE_HEIGTH 720
#define CAMERA_SAMPLE_PIXELS CAMERA_SAMPLE_WIDTH * CAMERA_SAMPLE_HEIGTH

class Camera
{
public:

    Camera( void );
    bool init( int id = CAMERA_ID_MAIN, int method = cv::CAP_V4L2  );

    bool read( void );
    bool read_raw( void );
    bool show( cv::Mat frame );
    void stop( void );

    int height;
    int width;

    cv::Mat frame;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

private:


    cv::VideoCapture camera;


    const static float camera_intrinsics[];
    const static float camera_distortion[];


};

#endif // CAMERA_H