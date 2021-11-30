#include <opencv2/opencv.hpp>
#include <iostream>
#include "camera.h"
#include "optical_flow.h"
#include "mavlink.h"

OpticalFlow flow;
Camera cam;

/* OPTICAL_FLOW_RAD #106
 * https://docs.px4.io/v1.12/en/sensor/optical_flow.html
 * https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD
 */
mavlink_optical_flow_rad_t message;



int main()
{
    bool streaming = true;
    float fx = .0f;
    float fy = .0f;
    int rate = 20;

    flow.init( fx, fy, rate, 1280, 720, 200 );

    if( !cam.init() )
        return -1;

    while(streaming){

        uint32_t img_time_us = cam.read();
        int dt = 0;
        float flow_x, flow_y = 0; 

        flow.compute_flow( cam.frame.data, img_time_us, dt, flow_x, flow_y );
        if( !cam.show( cam.frame ) ){
            streaming = false;
        }

    }

    cam.stop();

    return 0;
}

