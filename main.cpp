#include <opencv2/opencv.hpp>
#include <iostream>
#include "camera.h"
#include "optical_flow.h"
#include "mavlink.h"

#define CAMERA_WIDTH    1280    // OBS: Most be supported by camera hardware / gstreamer
#define CAMERA_HEIGHT   720     // --||--
#define CAMERA_RATE     60      // --||--
#define CAMERA_FOCAL_X  500 // Focal length of camera in x direction (pixels)
#define CAMERA_FOCAL_Y  500 // Focal length of camera in y direction (pixels)
 
#define APP_SAMPLE_RATE 20  // Rate of transmission of optical flow
#define APP_FEAUTURE_NUM 200  // Rate of transmission of optical flow

Camera cam;         // Camera object
OpticalFlow flow;   // OpticalFlow object

/* OPTICAL_FLOW_RAD #106
 * https://docs.px4.io/v1.12/en/sensor/optical_flow.html
 * https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD
 */
mavlink_optical_flow_rad_t msg; // Message struct for mavlink communication
cv::Mat image_8b = cv::Mat(CAMERA_WIDTH, CAMERA_HEIGHT, CV_8UC1); // Allocation of space for gray-scale image

float flow_x = 0.0;
float flow_y = 0.0;
int dt_us = 0; // Time between optical flow computations in microseconds
uint32_t img_time_us = 0;


void prepare_optical_flow_msg( float flow_x, float flow_y, int dt_us, int flow_quality, uint32_t img_time_stamp ){

    msg.time_usec = img_time_stamp;
    msg.sensor_id = 0; // ??? 
    msg.integration_time_us = dt_us;
    msg.integrated_x = - flow_y; 
    msg.integrated_y = flow_x; 
    msg.integrated_xgyro = 0.0;  // No gyro sensor onboard
    msg.integrated_ygyro = 0.0;  // No gyro sensor onboard
    msg.integrated_zgyro = 0.0;  // No gyro sensor onboard
    msg.temperature = 0.0;       // No temperature sensor onboard
    msg.quality = flow_quality;
    msg.time_delta_distance_us = 0.0; // ???
    msg.distance = -1.0; // Mark as invalid

}

int main()
{
    bool streaming = true;

    // Initialize camera class
    if( !cam.init( CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_RATE ) )
        return -1;

    // Initialize optical flow class
    flow.init( CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOCAL_X, CAMERA_FOCAL_Y, APP_SAMPLE_RATE, APP_FEAUTURE_NUM );


    while(streaming){
        // Read newest image from camera
        if( cam.read( img_time_us ) ){

            // Convert to gray-scale (needed for optical flow)
            cv::cvtColor( cam.image , image_8b, cv::COLOR_BGR2GRAY);

            // Compute flow from image, and save the values in flox_x and flow_y
            flow.compute_flow( image_8b, img_time_us, flow_x, flow_y, dt_us );

            printf("x: %.2f, y: %.2f \n", flow_x, flow_y);

            // Visualize the flow
            if( !cam.show( image_8b, 2 ) ){
                streaming = false;
            }
        }
    }

    cam.stop();

    return 0;
}

