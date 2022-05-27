/**
 * feature_tracker.h
 *
 * Created on: September 17, 2020
 * Author: SolidGeek
 */

#ifndef FEATURE_TRACKER_H_
#define FEATURE_TRACKER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <vector>
#include <iostream>


#define FEATURE_STATUS_INACTIVE 0 
#define FEATURE_STATUS_ACTIVE 1
#define FEATURE_STATUS_REQUEST 2

class FeatureTracker
{

public:

        FeatureTracker( void );

        void init( std::vector<int> &feature_status, int number_of_features, int threshold = 40 );

        /**
         * @brief Continuously extracts points on a grid from grayscale image and tracks them over time. 
         * 
         * @param img                   - The image in which to track features
         * @param features              - An array to hold the features tracked
         * @param feature_status        - An array to hold the status of each tracked feature
         * 
        */
        void track_features(const cv::Mat &img, std::vector<cv::Point2f> &features, std::vector<int> &feature_status );

        /**
         * @brief Loops through the feature status array, to see if any features has gone missing. If any is missing, request new features in its place by setting status[i] = 2.
         * 
         * @param feature_status - Array holding the status for each feature
         * 
        */
        void update_feature_status( std::vector<int> &feature_status );

private:

        cv::Mat prev_img;
        std::vector<cv::Point2f> prev_corners;
        std::vector<unsigned char> prev_status;

        std::vector<int> status;
        std::vector<cv::Point2f> features_current, features_previous;

        int threshold; 

        /**
         * @brief Counts the amount of requested features, and creates new in areas that are missing features.
         * 
         * @param img            - The image in which to find new features to track
         * @param features       - An array of the currently used features
         * @param feature_status - An array of the feature status. 0 == inactive, 1 == active, 2 == request new feature.
        */
        void init_more_points( const cv::Mat &img, std::vector<cv::Point2f> &features, std::vector<int> &feature_status );

        /**
         * @brief Compares the distance of each point in a vector "keypoints" to the currently active features, and saves the valid ones in the array "good". 
         * 
         * @param keypoints     - Original array of keypoints to be sorted
         * @param good          - A target array to hold the valid / good keypoints
         * @param unused        - Pointer to an array to hold unused keypoints. Pass a null pointer to discard the unused.
         * @param distance      - Max distanse from keypoints to current features
         * @param max_good      - The maxmial number of good keypoints to return
        */
        int find_valid_keypoints( std::vector<cv::KeyPoint> &keypoints, std::vector<cv::KeyPoint> &good, std::vector<cv::KeyPoint> * unused, unsigned int distance, int max_good );

};

#endif
