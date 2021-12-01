#include <opencv2/opencv.hpp>
#include <iostream>
#include "camera.h"
#include "optical_flow.h"
#include "mavlink.h"
#include "serial.h"
#include "timing.h"

#define CAMERA_WIDTH    1280    // OBS: Most be supported by camera hardware / gstreamer
#define CAMERA_HEIGHT   720     // --||--
#define CAMERA_RATE     60      // --||--
#define CAMERA_FOCAL_X  500 // Focal length of camera in x direction (pixels)
#define CAMERA_FOCAL_Y  500 // Focal length of camera in y direction (pixels)
 
#define OPTICALFLOW_OUTPUT_RATE 20  // Rate of transmission of optical flow
#define OPTICALFLOW_FEAUTURE_NUM 200  // Rate of transmission of optical flow

Serial uart( "/dev/ttyTHS1", SERIAL_WRITE ); 
Camera cam;         // Camera object
OpticalFlow flow;   // OpticalFlow object

// Allocation of space for gray-scale image
cv::Mat data = cv::Mat(CAMERA_WIDTH, CAMERA_HEIGHT, CV_8UC1);

float flow_x = 0.0;
float flow_y = 0.0;
int dt_us = 0;
uint32_t img_time_us = 0;


int main()
{
    bool streaming = true;

    uart.setup( SERIAL_TYPE_THS, B115200 );

    // Initialize camera class
    if( !cam.init( CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_RATE ) )
        return -1;

    // Initialize optical flow class
    flow.init( CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOCAL_X, CAMERA_FOCAL_Y, OPTICALFLOW_OUTPUT_RATE, OPTICALFLOW_FEAUTURE_NUM );


    while(streaming){
        // Read newest image from camera
        if( cam.read( img_time_us ) ){

            // Convert to gray-scale (needed for optical flow)
            cv::cvtColor( cam.image , data, cv::COLOR_BGR2GRAY);

            // Compute flow from image data, and save the values in flox_x and flow_y
            int flow_quality = flow.compute_flow( data, img_time_us, flow_x, flow_y, dt_us );

            if (flow_quality >= 0) {

                // Prepare mavlink package
                /* OPTICAL_FLOW_RAD #106
                 * https://docs.px4.io/v1.12/en/sensor/optical_flow.html
                 * https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD
                 */
                mavlink_optical_flow_rad_t flow_msg;

                flow_msg.time_usec = img_time_us;
                flow_msg.sensor_id = 0; // ??? 
                flow_msg.integration_time_us = dt_us;
                flow_msg.integrated_x = -flow_y; 
                flow_msg.integrated_y = flow_x; 
                flow_msg.integrated_xgyro = 0.0;  // No gyro sensor onboard
                flow_msg.integrated_ygyro = 0.0;  // No gyro sensor onboard
                flow_msg.integrated_zgyro = 0.0;  // No gyro sensor onboard
                flow_msg.temperature = 0.0;       // No temperature sensor onboard
                flow_msg.quality = flow_quality;
                flow_msg.time_delta_distance_us = 0.0; // ???
                flow_msg.distance = -1.0; // Mark as invalid

                mavlink_message_t message;
                mavlink_msg_optical_flow_rad_encode(0, 0, &message, &flow_msg );

                uint8_t buf[300];

                // Translate message to buffer
                unsigned len = mavlink_msg_to_send_buffer( buf, &message );

                // Write over uart
                uart.write_chars( buf, len );

            }

            // Visualize the flow
            /* if( !cam.show( image_8b, 2 ) ){
                streaming = false;
            }*/
        }
    }

    cam.stop();

    return 0;
}

