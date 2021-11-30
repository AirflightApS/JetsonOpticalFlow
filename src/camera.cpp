#include "camera.h"

const float Camera::camera_intrinsics[] = {962.7528696413754, 0, 609.7040640573265, 0, 962.9985208456596, 406.4304323047639, 0, 0, 1};
const float Camera::camera_distortion[] = {-0.3568698261604957, 0.186675202042404, -0.0003407870567855935, -0.0001591138431624921, -0.04288025847293331};


std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true";
}

Camera::Camera( void ){

    cameraMatrix = cv::Mat( cv::Size(3,3), CV_32F, (void*)camera_intrinsics );
    distCoeffs = cv::Mat(1,5, CV_32F, (void*)camera_distortion );

}

bool Camera::init( int id, int method  )
{

    std::string pipeline = gstreamer_pipeline(
        CAMERA_SAMPLE_WIDTH,CAMERA_SAMPLE_HEIGTH,
        CAMERA_SAMPLE_WIDTH,CAMERA_SAMPLE_HEIGTH,
        60, 0);

    camera.open( pipeline, cv::CAP_GSTREAMER );

    if( camera.isOpened()) {
        printf("Camera ready.");

        return true;

    }

    return false;
}



bool Camera::read()
{
    // Allocate Mat for processing the image.
    cv::Mat raw;

    // Read the camera into the raw variable
    camera.read( this->frame );

    if( !raw.empty() ){

        //cv::undistort( temp, undistored, cameraMatrix, distCoeffs );
        // Scale down the video by a factor of 2
        cv::resize(raw, this->frame, cv::Size(CAMERA_SAMPLE_WIDTH, CAMERA_SAMPLE_HEIGTH) );
    }
}

bool Camera::show( cv::Mat data ){

    static int frame_number = 0;
    char buffer[20];
    cv::Mat temp;

    cv::resize(data, temp, cv::Size(CAMERA_SAMPLE_WIDTH/2, CAMERA_SAMPLE_HEIGTH/2) );

    cv::imshow("Camera", temp );

    int key = cv::waitKey(10);

    if(key == 99){  // 'c' for capture
        sprintf(buffer, "images/frame-%d.jpg", frame_number);
        cv::imwrite( buffer, data );
        frame_number++;
    }
    else if (key == 27) // 'esc' to end
        return false;

    return true;
}

void Camera::stop()
{
    camera.release();
    cv::destroyAllWindows();
}
