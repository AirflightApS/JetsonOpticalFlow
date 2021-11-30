#include <opencv2/opencv.hpp>
#include <iostream>
#include "camera.h"
#include "feature_tracker.h"
#include "mavlink.h"


// Parameters used for featuretracker
std::vector<int> status;
std::vector<cv::Point2f> features_current, features_previous;

Camera cam;
FeatureTracker tracker;

mavlink_optical_flow_rad_t message;

int main()
{

    bool streaming = true;

    if( !cam.init() )
        return -1;

    // Initialize the tracker with 200 features
    tracker.init( status, 200 );

    while(streaming){

        cam.read();
        compute_optical_flow( cam.frame , 0.015);
        if( !cam.show( cam.frame ) ){
            streaming = false;
        }

    }

    cam.stop();

    return 0;
}

