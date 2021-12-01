#include "optical_flow.h"

OpticalFlow::OpticalFlow(){

    // Initialize the feature tracker
    tracker = new FeatureTracker();
}


void OpticalFlow::init( int img_width, int img_height, float f_x, float f_y, int output_rate, int num_feat, float conf_multi ){

    // Configure parameters
    focal_length_x = f_x;
    focal_length_y = f_y;
    output_rate = output_rate;
    image_width = img_width;
    image_height = img_height;
    num_features = num_feat;
    confidence_multiplier = conf_multi;

    // Initialize the tracker with number of desired features
    tracker->init( status_vector, num_features );
}

int OpticalFlow::compute_flow( cv::Mat image, const uint32_t img_time_us, float &flow_x, float &flow_y, int &dt_us ){

    // Variables
    float pixel_mean_x = 0.0, pixel_mean_y            = 0.0;
    float pixel_variance_x = 0.0, pixel_variance_y    = 0.0;
    float pixel_stddev_x = 0.0, pixel_stddev_y        = 0.0;
    float xsum = 0.0, ysum = 0.0;
    int feature_count = 0;
    float xsum_confidense = 0.0, ysum_confidense = 0.0;   // Sum of the features inside the confidense interval
    int confidense_count = 0;                       // Numbers of features inside the confidense interval
    int flow_quality = 0;


    // Track the current features in new frame
    tracker->track_features( image, features_current, status_vector );

    if (!features_current.empty() && !features_previous.empty()) {

        // Calculate sum of all pixel changes
		for (int i = 0; i < status_vector.size(); i++) {
			// Use only active features
			if ( status_vector[i] == FEATURE_STATUS_ACTIVE ) {
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
			for (int i = 0; i < status_vector.size(); i++) {
				if (status_vector[i] == FEATURE_STATUS_ACTIVE) {
					pixel_variance_x += pow(features_current[i].x - features_previous[i].x - pixel_mean_x, 2);
					pixel_variance_y += pow(features_current[i].y - features_previous[i].y - pixel_mean_y, 2);
				}
			}

            // Convert to standard deviation
			pixel_stddev_x = sqrt(pixel_variance_x / feature_count);
			pixel_stddev_y = sqrt(pixel_variance_y / feature_count);

            // Remove outliers based on standard deviation and the desired confidense interval
            for (int i = 0; i < status_vector.size(); i++) {
                if( status_vector[i] == FEATURE_STATUS_ACTIVE){
                    float delta_x = features_current[i].x - features_previous[i].x;
					float delta_y = features_current[i].y - features_previous[i].y;
                    int length = (int) ceil( sqrt( pow(delta_x,2) + pow(delta_y,2) ));
                    
                    // Check if inside confidence interval
					if ( (fabs(delta_x - pixel_mean_x) < pixel_stddev_x * confidence_multiplier && fabs(delta_y - pixel_mean_y) < pixel_stddev_y * confidence_multiplier) || length <= 1 ) {
						xsum_confidense += delta_x;
						ysum_confidense += delta_y;

                        // Visualize the flow in the frame
                        cv::line( image, cv::Point(features_previous[i].x, features_previous[i].y), cv::Point(features_current[i].x, features_current[i].y), cv::Scalar(255, 255, 255) );
                        cv::circle( image, cv::Point(features_current[i].x, features_current[i].y), 2, cv::Scalar(255, 255, 255), -1 );

						confidense_count++; 
					} 
                }
            }
        }
	}

    // Remember features
    features_previous = features_current;

    // Update the status of the newest features
    tracker->update_feature_status( status_vector );

    // Compute mean flow of the features in the confidense interval
    if( confidense_count ){
        flow_x = xsum_confidense / confidense_count;
        flow_y = ysum_confidense / confidense_count; 

        // Scale the flow from px/s to 1/s
        flow_x = atan2(flow_x, focal_length_x); //convert pixel flow to angular flow
        flow_y = atan2(flow_y, focal_length_y); //convert pixel flow to angular flow
    }else{
        flow_x = 0;
        flow_y = 0;
    }

    // Compute flow quality
    flow_quality = round(255.0 * confidense_count / status_vector.size());

    return flow_quality;

}

