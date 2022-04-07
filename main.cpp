#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include "camera.h"
#include "optical_flow.h"
#include "mavlink.h"
// #include "serial.h"
#include "timing.h"

#define CAMERA_WIDTH    1920    // OBS: Must be supported by camera hardware / gstreamer
#define CAMERA_HEIGHT   1080     // --||--
#define CAMERA_RATE     30      // --||--
#define CAMERA_FOCAL_X  654 // Focal length of camera in x direction (pixels) (655 for 77 FOV)
#define CAMERA_FOCAL_Y  654 // Focal length of camera in y direction (pixels) (655 for 77 FOV)
#define SCALE_FACTOR 4     // Reduce / scale down the image size to reduce processing time
        
#define OPTICAL_FLOW_OUTPUT_RATE 15   // Rate of transmission of optical flow
#define OPTICAL_FLOW_FEAUTURE_NUM 200 // Amount of features to track
#define SCALE_WIDTH CAMERA_WIDTH/SCALE_FACTOR
#define SCALE_HEIGHT CAMERA_HEIGHT/SCALE_FACTOR

#define CAMERA_SAMPLE_TIME  3.333e4 // in us, resulting in a rate of 30 Hz

// Serial uart( "/dev/ttyTHS1", SERIAL_WRITE ); 
Camera cam;         // Camera object
OpticalFlow flow;   // OpticalFlow object

bool app_active = true;

// Allocation of space for gray-scale image
cv::Mat frame = cv::Mat( SCALE_WIDTH, SCALE_HEIGHT, CV_8UC1 );
bool new_frame;
uint64_t frame_time_us = 0;

/**
 * @brief Samples camera at a constant frequency
 */
void camera_thread(){

    uint64_t last_frame_us = 0;
    uint64_t process_start_us = 0;
    uint32_t process_time_us = 0;
    
    // Initialize camera class and scale the image by factor: SCALE_FACTOR to increase sample time
    cam.init( CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_RATE, 2, SCALE_FACTOR );

    while( app_active ){

        // Time the processing time, to get a consistent frame frequency
        process_start_us = micros();

        // Read newest frame from camera
        if( cam.read( frame_time_us ) ){

            new_frame = true;
            // Convert color space and rescale image, saving the result in "frame"
            cv::cvtColor( cam.image, frame, cv::COLOR_BGR2GRAY );

        }

        last_frame_us = frame_time_us;
        process_time_us = micros() - process_start_us;

        usleep(CAMERA_SAMPLE_TIME - process_time_us);

    }
    
    // Stop the camera feed
    cam.stop();
}

/**
 * @brief Computes the optical flow
 */
void flow_thread(){

    int dt_us = 0; // Variable to hold the calculated delta time between accumulated optical flow measurements
    float flow_x = 0;
    float flow_y = 0;
    uint8_t flow_quality = 0;  

    mavlink_message_t message; // mavlink message header
    mavlink_optical_flow_rad_t flow_msg; // mavlink opticalflow package
    uint8_t buf[300]; // Buffer to hold outgoing mavlink package


    // Initialize optical flow class
    flow.init( SCALE_WIDTH, SCALE_HEIGHT, CAMERA_FOCAL_X, CAMERA_FOCAL_Y, OPTICAL_FLOW_OUTPUT_RATE, OPTICAL_FLOW_FEAUTURE_NUM );

    while( app_active ){

        if( new_frame ){
            
            // Compute flow from image data, and save the values in flox_x and flow_y
            int flow_quality = flow.compute_flow( frame, frame_time_us, flow_x, flow_y, dt_us );     

            // Rotate flow to match the orientation of PX4
            float flow_out_x = -flow_y; 
            float flow_out_y = flow_x;

            // Visualize the flow
            if( !cam.show( frame ) ){
                app_active = false;
            } 
            
            if (flow_quality >= 0) {

                // Prepare optical flow mavlink package
                flow_msg.time_usec = frame_time_us;
                flow_msg.integration_time_us = dt_us;
                flow_msg.integrated_x = flow_out_x; 
                flow_msg.integrated_y = flow_out_y; 
                flow_msg.quality = flow_quality;
                flow_msg.distance = -1; // distance_m; // Distance measured by LIDAR

                // Fill the mavlink message, with the optical flow package
                mavlink_msg_optical_flow_rad_encode(1, MAV_COMP_ID_PERIPHERAL, &message, &flow_msg );

                // Translate message to buffer
                unsigned len = mavlink_msg_to_send_buffer( buf, &message );

                // Write over uart
                // uart.write_chars( buf, len );

                printf("Sensor quality: %d, at %.2f Hz \t vx: %.2f \t vy: %.2f \n", flow_quality, 1.0e6f/dt_us, flow_out_x, flow_out_y );

            }

            new_frame = false; // Frame has been used, await new frame

        }

        // Sleep for 1 ms
        usleep(1e3);

    }
}


int main()
{

    // Prepare UART port
    // uart.setup( SERIAL_TYPE_THS, B921600 );

    printf("Ready");

    // Prepare multi-threading
    std::thread t1( camera_thread );
    std::thread t2( flow_thread );

    // Start multi-threading
    t1.join();
    t2.join();

    return 0;
}

