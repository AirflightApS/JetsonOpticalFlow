#include "optical_flow.h"

OpticalFlow::OpticalFlow(){

    // Initialize the feature tracker
    tracker = new FeatureTracker();
}


void OpticalFlow::init( float f_length_x, float f_length_y, int ouput_rate, int img_width, int img_height, int num_feat, float conf_multi ){

    set_focal_lengt_x( f_length_x );
    set_focal_lengt_y( f_length_y );
    set_output_rate( ouput_rate );
    set_image_width( img_width );
    set_image_height( img_height );

    // Initialize the tracker with number of desired features
    tracker->init( status, num_features );
}

int OpticalFlow::compute_flow( uint8_t *img_current, const uint32_t img_time_us, int &dt_us, float &flow_x, float &flow_y ){

    // Prepare static variable to hold image data
    static cv::Mat image = cv::Mat(image_height, image_width, CV_8UC1);

    // Variables
    float pixel_mean_x, pixel_mean_y = 0;
    float pixel_variance_x, pixel_variance_y = 0;
    float xsum, ysum = 0;
    int feature_count = 0;

    float xsum_confidense, ysum_confidense = 0;
    int confidense_count = 0;
    int flow_quality = 0;


    // Load image data into opencv Mat object
    image.data = (uchar *)img_current;


    // Track the current features in new frame
    tracker->track_features( image, features_current, status );

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

					if ( (fabs(val_x - pixel_mean_x) < pixel_variance_x * confidence_multiplier && fabs(val_y - pixel_mean_y) < pixel_variance_y * confidence_multiplier) || length <= 1 ) {
						xsum_confidense += val_x;
						ysum_confidense += val_y;

                        // Visualize the flow in the frame
                        cv::line( image, cv::Point(features_previous[i].x, features_previous[i].y), cv::Point(features_current[i].x, features_current[i].y), cv::Scalar(255, 255, 255) );
                        cv::circle( image, cv::Point(features_current[i].x, features_current[i].y), 2, cv::Scalar(255, 255, 255), -1 );

						confidense_count++; 
					} 
                }
            }
            // If some of the samples were inside the confidense interval
            if(confidense_count){
                xsum = xsum_confidense;
                ysum = ysum_confidense;
            }
        }
	}

    features_previous = features_current;

    // Update the status of the newest features
    tracker->update_feature_status( status );
    
    // Compute flow quality
    flow_quality = round(255.0 * confidense_count / status.size());

    // Compute mean flow
    flow_x = xsum_confidense / confidense_count;
    flow_y = ysum_confidense / confidense_count; 

    // Scale the flow from px/s to 1/s
    flow_x = atan2(flow_x, focal_length_x); //convert pixel flow to angular flow
	flow_y = atan2(flow_y, focal_length_y); //convert pixel flow to angular flow

}

