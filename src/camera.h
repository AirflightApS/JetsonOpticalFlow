#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <iostream>

#define CAMERA_ID_MAIN 0
#define CAMERA_ORIENTATION_UP = 0

class Camera
{
public:

    Camera( void );
    
    bool init( int width, int height, int frame_rate, int orientation = CAMERA_ORIENTATION_UP );
    void stop( void );
    bool read( void );
    bool show( cv::Mat image, float scale = 1 );
    
    // Variables
    int image_height;
    int image_width;
    cv::Mat image;

private:

    // Opencv VideoCapture object
    cv::VideoCapture capture;

};

#endif // CAMERA_H