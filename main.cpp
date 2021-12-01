#include <opencv2/opencv.hpp>
#include <iostream>
#include "camera.h"
#include "optical_flow.h"
#include "mavlink.h"

#define CAMERA_WIDTH    1280    // OBS: Most be supported by camera hardware / gstreamer
#define CAMERA_HEIGHT   720     // --||--
#define CAMERA_RATE     60      // --||--
#define CAMERA_FOCAL_X  520.23f // Focal length of camera in x direction (pixels)
#define CAMERA_FOCAL_Y  420.23f // Focal length of camera in y direction (pixels)
 
#define APP_SAMPLE_RATE 20  // Rate of transmission of optical flow
#define APP_FEAUTURE_NUM 200  // Rate of transmission of optical flow

Camera cam;         // Camera object
OpticalFlow flow;   // OpticalFlow object

/* OPTICAL_FLOW_RAD #106
 * https://docs.px4.io/v1.12/en/sensor/optical_flow.html
 * https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD
 */
mavlink_optical_flow_rad_t message; // Message struct for mavlink communication
cv::Mat image = cv::Mat(CAMERA_WIDTH, CAMERA_HEIGHT, cv::CV_8UC1); // Allocation of space for gray-scale image

float flow_x = 0.0;
float flow_y = 0.0;
int dt_us = 0; // Time between optical flow computations in microseconds


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
        uint32_t img_time_us = cam.read();

        // Convert to gray-scale (needed for optical flow)
        cv::cvtColor( cam.image , image, cv::COLOR_BGR2GRAY);

        // Compute flow from image, and save the values in flox_x and flow_y
        flow.compute_flow( image, img_time_us, dt, flow_x, flow_y );

        // Visualize the flow
        if( !cam.show( image ) ){
            streaming = false;
        }

    }

    cam.stop();

    return 0;
}

