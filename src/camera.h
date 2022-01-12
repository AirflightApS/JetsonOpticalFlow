#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "timing.h"

#define CAMERA_ID_MAIN 0
#define CAMERA_ORIENTATION_UP 0
#define CAMERA_ORIENTATION_LEFT 1

class Camera
{
public:

    Camera( void );
    
    bool init( int width, int height, int frame_rate, int orientation = CAMERA_ORIENTATION_LEFT, int scale = 1 );
    void stop( void );
    bool read( uint64_t &sample_time );
    bool show( cv::Mat image, int scale = 1 );
    
    // Variables
    int image_height;
    int image_width;
    cv::Mat image;

private:

    // Opencv VideoCapture object
    cv::VideoCapture capture;

};

#endif // CAMERA_H