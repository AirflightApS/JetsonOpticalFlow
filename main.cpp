#include <opencv2/opencv.hpp>
#include <iostream>
#include "camera.h"
#include "optical_flow.h"
#include "mavlink.h"
#include "serial.h"
#include "timing.h"

#define CAMERA_WIDTH    1280    // OBS: Must be supported by camera hardware / gstreamer
#define CAMERA_HEIGHT   720     // --||--
#define CAMERA_RATE     60      // --||--
#define CAMERA_FOCAL_X  500 // Focal length of camera in x direction (pixels)
#define CAMERA_FOCAL_Y  500 // Focal length of camera in y direction (pixels)
#define SCALE_FACTOR 4      // Reduce / scale down the image size to reduce processing time
        
#define OPTICAL_FLOW_OUTPUT_RATE 15   // Rate of transmission of optical flow
#define OPTICAL_FLOW_FEAUTURE_NUM 100 // Rate of transmission of optical flow
#define SCALE_WIDTH CAMERA_WIDTH/SCALE_FACTOR
#define SCALE_HEIGHT CAMERA_HEIGHT/SCALE_FACTOR


Serial uart( "/dev/ttyTHS1", SERIAL_WRITE ); 
Camera cam;         // Camera object
OpticalFlow flow;   // OpticalFlow object

// Allocation of space for gray-scale image
cv::Mat gray = cv::Mat( SCALE_WIDTH, SCALE_HEIGHT, CV_8UC1 );

// Variables to hold data from optical flow and camera
float flow_x = 0.0;
float flow_y = 0.0;
int dt_us = 0;
uint64_t img_time_us = 0;

bool active = true;


int main()
{
    uart.setup( SERIAL_TYPE_THS, B115200 );

    // Initialize camera class
    // Use gstreamer to scale the image by factor: SCALE_FACTOR
    if( !cam.init( CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_RATE, 0, SCALE_FACTOR ) )
        return -1;

    // Initialize optical flow class
    flow.init( SCALE_WIDTH, SCALE_HEIGHT, CAMERA_FOCAL_X, CAMERA_FOCAL_Y, OPTICAL_FLOW_OUTPUT_RATE, OPTICAL_FLOW_FEAUTURE_NUM );


    while(active){
        // Read newest image from camera
        if( cam.read( img_time_us ) ){

            // Convert color space and rescale image
            cv::cvtColor( cam.image , gray, cv::COLOR_BGR2GRAY);

            // Compute flow from image data, and save the values in flox_x and flow_y
            int flow_quality = flow.compute_flow( gray, img_time_us, flow_x, flow_y, dt_us );
            
           
            if (flow_quality >= 0) {

                // Buffer to hold outgoing mavlink package
                uint8_t buf[300];

                // Prepare optical flow mavlink package
                mavlink_optical_flow_rad_t flow_msg;
                flow_msg.time_usec = img_time_us;
                flow_msg.integration_time_us = dt_us;
                flow_msg.integrated_x = -flow_y; 
                flow_msg.integrated_y = flow_x; 
                flow_msg.quality = flow_quality;
                flow_msg.distance = -1.0; // No distance sensor (use onboard) 

                // Prepare mavlink message header
                mavlink_message_t message;

                // Fill the message, with the optical flow package
                mavlink_msg_optical_flow_rad_encode(1, MAV_COMP_ID_PERIPHERAL, &message, &flow_msg );

                // Translate message to buffer
                unsigned len = mavlink_msg_to_send_buffer( buf, &message );

                // Write over uart
                uart.write_chars( buf, len );
                
                printf("Sensor quality: %d, at %.2f Hz \n", flow_quality, 1.0e6f/dt_us );

            }

            // Visualize the flow
            if( !cam.show( gray ) ){
                active = false;
            }
           
        }
    }

    cam.stop();

    return 0;
}

