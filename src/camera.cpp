#include "camera.h"


// aelock: lock auto exposure - we want auto exposure, in low light conditions
// awblock: lock auto white balance - we dont want auto white balance as it changes the light intensity between frames

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc awblock=false aelock=false ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true";
}

Camera::Camera( void ){}

bool Camera::init( int width, int height, int frame_rate, int orientation, int scale )
{

    image_width = width;
    image_height = height;

    std::string pipeline = gstreamer_pipeline( width, height, width/scale, height/scale, frame_rate, orientation);

    capture.open( pipeline, cv::CAP_GSTREAMER );

    if( capture.isOpened()) {
        printf("Camera ready.");

        return true;
    }

    return false;
}



bool Camera::read( uint64_t &sample_time )
{
    // Allocate Mat for processing the image.
    cv::Mat raw;

    // Read the camera into the raw variable
    capture.read( raw );
    sample_time = micros();

    if( !raw.empty() ){
        this->image = raw.clone();
        return true;
    }
        

    return false;
}

bool Camera::show( cv::Mat data, int scale ){

    static int frame_number = 0;
    cv::Mat temp = cv::Mat(image_height, image_width, CV_8UC1);
    char buffer[20];

    if( scale != 1 )
        cv::resize( data, temp, cv::Size( image_width/scale, image_height/scale ) );
    else
        temp = data.clone();

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
    capture.release();
    cv::destroyAllWindows();
}
