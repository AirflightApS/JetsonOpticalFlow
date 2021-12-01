/**
 * feature_tracker.cpp
 *
 * Created on: December 1, 2021
 * Author: SolidGeek
 */

#include <opencv2/opencv.hpp>
#include <stdint.h>
#include "feature_tracker.h"

#define DEFAULT_CONFIDENCE_MULTIPLIER 1.282f // 80% confidence interval

class OpticalFlow
{

public:

    /**
     * @brief Construct a new Optical Flow object
     */
    OpticalFlow( void ) ;

    /**
     * @brief Initialize the Optical flow class (should be called at startup)
     * 
     * @param f_x           Focal length of camera in x-axis
     * @param f_y           Focal length of camera in y-axis
     * @param ouput_rate    Desired output rate
     * @param img_width     Width of image 
     * @param img_height    Height of image
     * @param num_feat      Desired number of features
     * @param conf_multi    Desired confidense interval for outlier rejection
     */
    void init( int img_width, int img_height, float f_x, float f_y, int output_rate, int num_feat, float conf_multi = DEFAULT_CONFIDENCE_MULTIPLIER );

    /**
     * @brief 
     * 
     * @param img_current   Data of the newest image
     * @param img_time_us   Timestamp of picture capture
     * @param dt_us         Delta time since last picture
     * @param flow_x        Flow in x-dir
     * @param flow_y        Flow in y-dir
     * @return int          Computed flow quality
     */
    int compute_flow( cv::Mat img_current, const uint32_t img_time_us, float &flow_x, float &flow_y, int &dt_us );

private:

    int rate_limit(int flow_quality, const uint32_t img_time_us, int *dt_us, float *flow_x, float *flow_y);

    // Configuration
	int image_width;
	int image_height;
	float focal_length_x;
	float focal_length_y;
	int output_rate;

    // Params for feature tracking
    FeatureTracker * tracker;
    
	int num_features;
	float confidence_multiplier;
    std::vector<int> status_vector; // Vector of features status
    std::vector<cv::Point2f> features_current; // Newest 
     std::vector<cv::Point2f> features_previous;
};

