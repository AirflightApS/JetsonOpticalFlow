#include <opencv2/opencv.hpp>
#include <stdint.h>
#include "feature_tracker.h"

#define DEFAULT_CONFIDENCE_MULTIPLIER 1.645f // 90% confidence interval

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
     * @param f_length_x    Focal length of camera in x-axis
     * @param f_length_y    Focal length of camera in y-axis
     * @param ouput_rate    Desired output rate
     * @param img_width     Width of image 
     * @param img_height    Height of image
     * @param num_feat      Desired number of features
     * @param conf_multi    Desired confidense interval for outlier rejection
     */
    void init( float f_length_x, float f_length_y, int ouput_rate, int img_width, int img_height, int num_feat, float conf_multi = DEFAULT_CONFIDENCE_MULTIPLIER );

    /**
     * @brief 
     * 
     * @param img_current   
     * @param img_time_us 
     * @param dt_us 
     * @param flow_x 
     * @param flow_y 
     * @return int 
     */
    int compute_flow( uint8_t *img_current, const uint32_t &img_time_us, int &dt_us, float &flow_x, float &flow_y );


    inline void set_image_width(int value) { image_width = value; };
	inline void set_image_height(int value) { image_height = value; };
	inline void set_focal_lengt_x(float value) { focal_length_x = value; };
	inline void set_focal_lengt_y(float value) { focal_length_y = value; };
    inline void set_output_rate( int value ) { output_rate = value; };

private:

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
    std::vector<int> status;
    std::vector<cv::Point2f> features_current, features_previous;

};

