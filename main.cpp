#include <opencv2/opencv.hpp>
#include <iostream>
#include "camera.h"
#include "feature_tracker.h"


// Parameters used for featuretracker
std::vector<int> status;
std::vector<cv::Point2f> features_current, features_previous;

Camera cam;
FeatureTracker tracker;

void compute_optical_flow( cv::Mat frame, float dt ){

    // Variables
    float pixel_mean_x, pixel_mean_y = 0;
    float pixel_variance_x, pixel_variance_y = 0;
    float xsum, ysum = 0;
    int feature_count = 0;

    float xsum_confidense, ysum_confidense = 0;
    int confidense_count = 0;
    float z_multiplier = 1.28; // 80% confidense interval 
    int flow_quality = 0;


    // Track the current features in new frame
    tracker.track_features( frame, features_current, status );

    if (!features_current.empty() && !features_previous.empty()) {

        // Calculate sum of all pixel changes
		for (int i = 0; i < status.size(); i++) {
			// Use only active features
			if ( status[i] == 1 ) {
				xsum += features_current[i].x - features_previous[i].x;
				ysum += features_current[i].y - features_previous[i].y;

				feature_count++;
			}
		}

        if( feature_count ){
            // Calculate pixel flow means
            pixel_mean_x = xsum / feature_count;
            pixel_mean_y = ysum / feature_count;

            pixel_variance_x = 0;
            pixel_variance_y = 0;

            // Calculate sample variance (sum)
			for (int i = 0; i < status.size(); i++) {
				if (status[i] == 1) {
					pixel_variance_x += pow(features_current[i].x - features_previous[i].x - pixel_mean_x, 2);
					pixel_variance_y += pow(features_current[i].y - features_previous[i].y - pixel_mean_y, 2);
				}
			}

            // Convert to standard deviation
			pixel_variance_x = sqrt(pixel_variance_x / feature_count);
			pixel_variance_y = sqrt(pixel_variance_y / feature_count);

            // Remove outliers based on standard deviation and the desired confidense interval
            for (int i = 0; i < status.size(); i++) {
                if( status[i] == 1){
                    float val_x = features_current[i].x - features_previous[i].x;
					float val_y = features_current[i].y - features_previous[i].y;
					//check if inside confidence interval

                    int length = (int) ceil( sqrt( pow(val_x,2) + pow(val_y,2) ));

					if ( (fabs(val_x - pixel_mean_x) < pixel_variance_x * z_multiplier && fabs(val_y - pixel_mean_y) < pixel_variance_y * z_multiplier) || length <= 1 ) {
						xsum_confidense += val_x;
						ysum_confidense += val_y;

                        // Visualize the flow in the frame
                        cv::line( frame, cv::Point(features_previous[i].x, features_previous[i].y), cv::Point(features_current[i].x, features_current[i].y), cv::Scalar(255, 255, 255) );
                        cv::circle( frame, cv::Point(features_current[i].x, features_current[i].y), 2, cv::Scalar(255, 255, 255), -1 );

						confidense_count++; 
					} 
                }
            }
            // If some of the samples were inside the confidense interval
            if(confidense_count){
                xsum = xsum_confidense;
                ysum = ysum_confidense;
                count = confidense_count;
            }
        }
	}

    features_previous = features_current;

    // Update the status of the newest features
    tracker.update_feature_status( status );
    
    // Compute flow quality
    flow_quality = round(255.0 * confidense_count / status.size());

}


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

