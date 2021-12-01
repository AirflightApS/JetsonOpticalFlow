#include "camera.h"


std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true";
}

Camera::Camera( void ){}

bool Camera::init( int width, int height, int frame_rate, int orientation )
{
    std::string pipeline = gstreamer_pipeline( width, height, width, height, frame_rate, orientation);

    camera.open( pipeline, cv::CAP_GSTREAMER );

    if( camera.isOpened()) {
        printf("Camera ready.");

        return true;
    }

    return false;
}



bool Camera::read( void )
{
    // Allocate Mat for processing the image.
    cv::Mat raw;

    // Read the camera into the raw variable
    capture.read( raw );

    if( !raw.empty() )
        return true;

    return false;
}

bool Camera::show( cv::Mat data, float scale ){

    static int frame_number = 0;
    cv::Mat temp = cv::Mat(image_height, image_width, CV_8UC1);
    char buffer[20];

    if( scale != 1 )
        cv::resize( data, temp, cv::Size( image_width/scale, image_height/scale ) );
    else
        temp = data;

    // Show image using opencv image container
    cv::imshow("Camera", temp );

    // Listen for any key-presses
    int key = cv::waitKey(10);

    // Taking a snapshot and saving it to the device
    if(key == 99){  // 'c' for capture
        sprintf(buffer, "images/frame-%d.jpg", frame_number);
        cv::imwrite( buffer, data );
        frame_number++;
    }
    // Return false, to exit application
    else if (key == 27) // 'esc' to end
        return false;

    return true;
}

void Camera::stop()
{
    camera.release();
    cv::destroyAllWindows();
}
