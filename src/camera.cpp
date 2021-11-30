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
        120, 0);

    camera.open( pipeline, cv::CAP_GSTREAMER );

    // For raw data
    // camera.open( id , method );

    if( camera.isOpened()) {

        // Configuring opencv video object
        // Format is Bayer 10 bit = RG10
        /* For raw data
        int format = cv::VideoWriter::fourcc('R','G','1','0');
        camera.set(cv::CAP_PROP_FOURCC, format);
        camera.set(cv::CAP_PROP_CONVERT_RGB, 0);  // Stop opencv to try and demosaic the bayer by it self.
        camera.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_SAMPLE_WIDTH);
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_SAMPLE_HEIGTH);

        // Reset the camera to default before setting desired params.
        system("v4l2-ctl --set-ctrl=exposure=2495"); 
        system("v4l2-ctl --set-ctrl=gain=16");
        camera.read(frame); // Read a frame to expose the settings

        // Configure camera using v4l2lib 
        system("v4l2-ctl --set-ctrl=bypass_mode=0");  // Must be used for v4l2
        system("v4l2-ctl --set-ctrl=sensor_mode=5");  // 5 = 1280x720
        system("v4l2-ctl --set-ctrl=exposure=40000"); 
        system("v4l2-ctl --set-ctrl=gain=150");
        */

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
    camera.read( raw );

    if( !raw.empty() ){

        //cv::undistort( temp, undistored, cameraMatrix, distCoeffs );
        // Scale down the video by a factor of 2
        cv::resize(raw, this->frame, cv::Size(CAMERA_SAMPLE_WIDTH/2, CAMERA_SAMPLE_HEIGTH/2) );
    }
}


bool Camera::read_raw()
{
    // Allocate Mat for processing the image.
    cv::Mat raw8b;
    cv::Mat bayer10b( CAMERA_SAMPLE_HEIGTH, CAMERA_SAMPLE_WIDTH, CV_16UC1 );
    cv::Mat temp( CAMERA_SAMPLE_HEIGTH, CAMERA_SAMPLE_WIDTH, CV_16UC1 );
    cv::Mat undistored( CAMERA_SAMPLE_HEIGTH, CAMERA_SAMPLE_WIDTH, CV_8UC1 );

    // Read the camera into the raw variable
    camera.read( raw8b );

    if( !raw8b.empty() ){
        // Process the raw 8bit data to become 16bit (each with 10bit color information)
        // Loop through all data in raw, and build buffer of 10bit bayer (red: 3b, green: 4b, blue: 3b)
        for( uint32_t i = 0; i < CAMERA_SAMPLE_PIXELS; i++ ){

            // byte order from raw: lowbyte, highbyte 
            uint8_t lsb = raw8b.at<uint8_t>(0, i*2);
            uint8_t msb = raw8b.at<uint8_t>(0, i*2+1);
            uint16_t data = (msb<<8) | lsb;

            bayer10b.at<uint16_t>(0, i) = data;
        }

        // Decode the Bayer data to BGR using opencv2, pattern is BG 
        cv::cvtColor( bayer10b, temp, cv::COLOR_BayerBG2GRAY );

        // cv::multiply( temp, cv::Scalar(1, 0.7, 1), temp );

        // Scale (10bit) channels to 8bit: 1024/256 = 0.25
        // Save the decoded image to "frame"
        cv::convertScaleAbs( temp, temp, 0.25 );

        //cv::undistort( temp, undistored, cameraMatrix, distCoeffs );
        // Scale down the video by a factor of 2
        cv::resize(temp, this->frame, cv::Size(CAMERA_SAMPLE_WIDTH/2, CAMERA_SAMPLE_HEIGTH/2) );
    }
}

bool Camera::show( cv::Mat data ){

    static int frame_number = 0;
    char buffer[20];

    cv::imshow("Camera", data );

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
