#include "optical_flow.h"

OpticalFlow::OpticalFlow(){

    // Initialize the feature tracker
    tracker = new FeatureTracker();

    camera_matrix.create(3, 3);
	camera_distortion.create(1, 5);
}


void OpticalFlow::init( int img_width, int img_height, float f_x, float f_y, int rate, int num_feat, float conf_multi ){

    // Configure parameters
    focal_length_x = f_x;
    focal_length_y = f_y;
    output_rate = rate;
    image_width = img_width;
    image_height = img_height;
    num_features = num_feat;
    confidence_multiplier = conf_multi;

    // Initialize the tracker with number of desired features
    // 3td parameter is threshold for FAST algorithm. Lower value = more noisy features
    tracker->init( status_vector, num_features, 10 );
}


void OpticalFlow::set_camera_matrix(float focal_len_x, float focal_len_y, float principal_point_x, float principal_point_y)
{
	camera_matrix <<    focal_len_x, 0.0f, principal_point_x,
		                0.0f, focal_len_y, principal_point_y,
		                0.0f, 0.0f, 1.0f;

	isset_camera_matrix = true;
}

void OpticalFlow::set_camera_distortion( float p1, float p2, float k1, float k2, float k3 )
{
	camera_distortion <<   k1, k2, p1, p2, k3;

	isset_camera_distortion = true;
}


int OpticalFlow::compute_flow( cv::Mat image, const uint64_t img_time_us, float &flow_x, float &flow_y, int &dt_us ){

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


    /* if (isset_camera_matrix && isset_camera_distortion) {
        features_temp = features_current;
        cv::undistortPoints(features_temp, features_current, camera_matrix, camera_distortion);

        // cv::undistortPoints returns normalized coordinates... -> convert
        for (int i = 0; i < num_features; i++) {
            features_current[i].x = features_current[i].x * camera_matrix(0, 0) +
                        camera_matrix(0, 2);
            features_current[i].y = features_current[i].y * camera_matrix(1, 1) +
                        camera_matrix(1, 2);
        }
    } */

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
    }else{
        flow_x = 0;
        flow_y = 0;
    }

    // Compute flow quality
    flow_quality = round(255.0 * confidense_count / status_vector.size());

    // Limit rate, by accumulating flow and taking the average quality. 
    flow_quality = rate_limit( flow_quality, img_time_us, &dt_us, &flow_x, &flow_y );

    // Scale the flow from px/s to 1/s
    flow_x = atan2(flow_x, focal_length_x); //convert pixel flow to angular flow
    flow_y = atan2(flow_y, focal_length_y); //convert pixel flow to angular flow

    return flow_quality;

}



int OpticalFlow::rate_limit(int flow_quality, const uint64_t image_time_us, int *dt_us, float *flow_x, float *flow_y)
{

	static uint64_t last_report = 0;
    static float sum_flow_x = 0;
    static float sum_flow_y = 0;
    static float sum_flow_quality = 0;
    static float valid_image_count = 0;

    // If the output rate is not configured, don't limit the rate
	if (output_rate <= 0) { 
		*dt_us = image_time_us - last_report;
		last_report = image_time_us;
		return flow_quality;
	}

    // If the flow quality if higher than 0, add the current flow to the sum.
	if (flow_quality > 0) {
		sum_flow_x += *flow_x;
		sum_flow_y += *flow_y;
		sum_flow_quality += flow_quality;
		valid_image_count++;
	}

	// limit rate according to parameter ouput_rate
	if ((image_time_us - last_report) >= (1.0e6f / output_rate)) {

		int average_flow_quality = 0;

		// Average the flow over the period since last update
		if (valid_image_count > 0) {
			average_flow_quality = std::floor(sum_flow_quality / valid_image_count);
		}

        // Set the output flow equal to the accumulated flow (in pixels)
		*flow_x = sum_flow_x;
		*flow_y = sum_flow_y;

		// Reset variables
		sum_flow_y = 0; sum_flow_x = 0; 
        sum_flow_quality = 0;
        valid_image_count = 0;

        // Calculate delta time
		*dt_us = image_time_us - last_report;
		last_report = image_time_us;

		return average_flow_quality;

	} else {

		return -1; // Signaling that it should not yet report the values

	}

}