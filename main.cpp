#include <opencv2/opencv.hpp>
#include <iostream>
#include "camera.h"
#include "feature_tracker.h"


// Parameters used for featuretracker
std::vector<int> status;
std::vector<cv::Point2f> features_current, features_previous;

Camera cam;
FeatureTracker track;
void compute_sparse_flow( cv::Mat frame, float dt ){

    float xsum, ysum = 0;
    int count = 0;

    // Track features in new frame
    track.track_features( frame, features_current, status );

    if (!features_current.empty() && !features_previous.empty()) {

        // Calculate pixel flow

		for (int i = 0; i < status.size(); i++) {
			//just use active features
			if ( status[i] == 1 ) {
				xsum += features_current[i].x - features_previous[i].x;
				ysum += features_current[i].y - features_previous[i].y;

				count++;
			}
		}

        if( count ){
            // Calculate pixel flow means
            float pixel_mean_x = xsum / count;
            float pixel_mean_y = ysum / count;

            float pixel_variance_x = 0;
            float pixel_variance_y = 0;

            // Calculate sample variance (sum)
			for (int i = 0; i < status.size(); i++) {
				if (status[i] == 1) {
					pixel_variance_x += pow(features_current[i].x - features_previous[i].x - pixel_mean_x, 2);
					pixel_variance_y += pow(features_current[i].y - features_previous[i].y - pixel_mean_y, 2);
				}
			}

            // Convert to standard deviation
			pixel_variance_x = sqrt(pixel_variance_x / count);
			pixel_variance_y = sqrt(pixel_variance_y / count);

            // Now remove outliers based on standard deviation and a confidense interval of 90%
            float z_multiplier = 1.65; // 90% confidense interval 

            float xsum_confidense = 0;
            float ysum_confidense = 0;
            int count_confidense = 0;

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

						count_confidense++; 
					} 
                }
            }
            // If some of the samples were inside the confidense interval
            if(count_confidense){
                xsum = xsum_confidense;
                ysum = ysum_confidense;
                count = count_confidense;
            }
        }
	}

    features_previous = features_current;

    track.update_feature_status( status );
    
}


int main()
{

    bool streaming = true;

    if( !cam.init() )
        return -1;

    track.init( status, 200 );

    while(streaming){

        cam.read();
        compute_sparse_flow( cam.frame , 0.015);
        if( !cam.show( cam.frame ) ){
            streaming = false;
        }

    }

    cam.stop();

    return 0;
}

