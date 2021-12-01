/**
 * feature_tracker.cpp
 *
 * Created on: December 1, 2021
 * Author: SolidGeek
 */

#include "feature_tracker.h"

bool compare_keypoints(const cv::KeyPoint &first, const cv::KeyPoint &second) {
    return first.response > second.response;
}

FeatureTracker::FeatureTracker( void ){}

void FeatureTracker::init( std::vector<int> &feature_status, int number_of_features ){

    if (feature_status.empty()) {
		feature_status.resize( number_of_features, FEATURE_STATUS_REQUEST );
	}

    this->prev_status.resize( number_of_features, FEATURE_STATUS_INACTIVE );

}

// corners (features, z_all_r) and status are output variables
void FeatureTracker::track_features(const cv::Mat &img, std::vector<cv::Point2f> &features, std::vector<int> &feature_status ) {
    if (!img.data)
        throw "Image is invalid";

    std::vector<unsigned char> status; // Vector to hold status of each tracked feature from calcOpticalFlowPyrLK
    std::vector<cv::Point2f> corners;  // Vector to hold new corner positions tracked by calcOpticalFlowPyrLK
    std::vector<float> errors;         // not used

    unsigned int num_features = feature_status.size();

    // Resize features array to size of status, and fill with unvalid points (outside frame)
    features.resize(num_features);
    std::fill(features.begin(), features.end(), cv::Point2f(-100, -100));


    // NEED COMMENT
    for (size_t i = 0; i < feature_status.size() && i < num_features; ++i) {
        if (feature_status[i] == FEATURE_STATUS_ACTIVE) {
            prev_status[i] = FEATURE_STATUS_ACTIVE;
        } else {
            prev_status[i] = FEATURE_STATUS_INACTIVE;  // if feature_status[i] == 0 feature is inactive, == 2 request new feature
        }
    }

    if (!prev_img.empty()) {
        if (!prev_corners.empty()) {

            cv::calcOpticalFlowPyrLK(prev_img, img, prev_corners, corners, status, errors, cv::Size(21, 21), 3);
            prev_corners = corners;

            for (size_t i = 0; i < prev_corners.size() && i < num_features; ++i) {
                // Check if calcOpticalFlowPyrLK lost track of a feature.
                if (!(prev_status[i] && status[i]))
                    prev_status[i] = FEATURE_STATUS_INACTIVE;

                // If last corner was a active feature, check if it is still valid (within the frame)
                if (prev_status[i] == 1) {
                    if (prev_corners[i].x < 0 || prev_corners[i].x > img.cols || prev_corners[i].y < 0 || prev_corners[i].y > img.rows ) {
                        feature_status[i] = FEATURE_STATUS_INACTIVE;
                    } else {
                        // Corner is still a feature, save it by setting feature_status[i] == 1 (active feature)
                        features[i] = prev_corners[i];
                        feature_status[i] = FEATURE_STATUS_ACTIVE;
                    }
                } else {
                    if (feature_status[i] == 1)  // be careful not to overwrite feature requests in feature_status
                        feature_status[i] = FEATURE_STATUS_INACTIVE;
                }
            }
        }
    }

    img.copyTo(prev_img);
    
    // initialize new points if needed
    init_more_points(img, features, feature_status );

}

void FeatureTracker::update_feature_status( std::vector<int> &feature_status ){

    //update feature status
	for (int i = 0; i < feature_status.size(); i++) {
		//new and now active
		if (feature_status[i] == FEATURE_STATUS_REQUEST) {
			feature_status[i] = FEATURE_STATUS_ACTIVE;
		}

		//inactive
		if (feature_status[i] == FEATURE_STATUS_INACTIVE) {
			feature_status[i] = FEATURE_STATUS_REQUEST;
		}
	}
}

int FeatureTracker::find_valid_keypoints( std::vector<cv::KeyPoint> &keypoints, std::vector<cv::KeyPoint> &good, std::vector<cv::KeyPoint> * unused, unsigned int distance, int max_good ){

    // Check if the new features are far enough from existing points
    int newPtIdx = 0;
    for (; newPtIdx < keypoints.size(); newPtIdx++) {
        int new_pt_x = keypoints[newPtIdx].pt.x;
        int new_pt_y = keypoints[newPtIdx].pt.y;

        bool far_enough = true;
        for (int j = 0; j < prev_corners.size(); j++) {
            if (prev_status[j] == FEATURE_STATUS_INACTIVE)
                continue;
            int existing_pt_x = prev_corners[j].x;
            int existing_pt_y = prev_corners[j].y;
            if (abs(existing_pt_x - new_pt_x) < distance && abs(existing_pt_y - new_pt_y) < distance) {
                far_enough = false;
                if( unused != NULL)
                    unused->push_back(keypoints[newPtIdx]);
                break;
            }
        }
        if (far_enough) {
            // Check if the new feature is too close to a new one
            for (int j = 0; j < good.size(); j++) {
                int existing_pt_x = good[j].pt.x;
                int existing_pt_y = good[j].pt.y;
                if (abs(existing_pt_x - new_pt_x) < distance && abs(existing_pt_y - new_pt_y) < distance) {
                    far_enough = false;
                    if( unused != NULL)
                        unused->push_back(keypoints[newPtIdx]);
                    break;
                }
            }
            if (far_enough) {
                good.push_back(keypoints[newPtIdx]);
                if (good.size() == max_good)
                    break;
            }
        }
    }

    return newPtIdx;
}

void FeatureTracker::init_more_points(const cv::Mat &img, std::vector<cv::Point2f> &features, std::vector<int> &feature_status ) {
    if (!img.data)
        throw "Image is invalid";

    unsigned int targetNumPoints = 0;

    // Count the features that need to be initialized
    for (int i = 0; i < feature_status.size(); i++) {
        if (feature_status[i] == FEATURE_STATUS_REQUEST )  // 2 means new feature requested
            targetNumPoints++;
    }

    // If no features is missing return.
    if (!targetNumPoints)
        return;

    int numBinsX = 4;
    int numBinsY = 4;
    int binWidth = img.cols / numBinsX;
    int binHeight = img.rows / numBinsY;
    int targetFeaturesPerBin = (feature_status.size() - 1) / (numBinsX * numBinsY) + 1;  // Total number of features that should be in each bin
    unsigned int dist = binWidth / targetFeaturesPerBin; // Minimal distance between two features 

    // Prepare vectors to hold keypoints
    std::vector<cv::KeyPoint> goodKeypoints, unusedKeypoints;

    // Prepare 4x4 matrix each cell with a zero
    std::vector<std::vector<int>> featuresPerBin(numBinsX, std::vector<int>(numBinsY, 0));

    // Count the number of active features in each bin
    for (int i = 0; i < prev_corners.size(); i++) {
        if (feature_status[i] == FEATURE_STATUS_ACTIVE) {
            int binX = prev_corners[i].x / binWidth;
            int binY = prev_corners[i].y / binHeight;

            if (binX >= numBinsX) {
                printf("Warning: writing to binX out of bounds: %d >= %d\n", binX, numBinsX);
                continue;
            }
            if (binY >= numBinsY) {
                printf("Warning: writing to binY out of bounds: %d >= %d\n", binY, numBinsY);
                continue;
            }
            featuresPerBin[binX][binY]++;
        }
    }

    // Go through each cell and detect features
    for (int x = 0; x < numBinsX; x++) {
        for (int y = 0; y < numBinsY; y++) {
            // Calculated missing features in bin[x][y]
            int neededFeatures = std::max(0, targetFeaturesPerBin - featuresPerBin[x][y]);

            // If any features / points are missing
            if (neededFeatures) {
                // Determine region the feature should be placed in.
                int col_from = x * binWidth;
                int col_to = std::min((x + 1) * binWidth, img.cols);
                int row_from = y * binHeight;
                int row_to = std::min((y + 1) * binHeight, img.rows);

                std::vector<cv::KeyPoint> keypoints, goodKeypointsBin;

                // Detect corners in section using FAST method, and save them in keypoints.
                cv::FAST( img.rowRange(row_from, row_to).colRange(col_from, col_to), keypoints, 10 );

                // Sort keypoints based on their "response"
                sort(keypoints.begin(), keypoints.end(), compare_keypoints);

                // Add bin offsets to the points (because we limitied FAST to on of the bin regions)
                for (int i = 0; i < keypoints.size(); i++) {
                    keypoints[i].pt.x += col_from;
                    keypoints[i].pt.y += row_from;
                }

                // Check if the new features are far enough from existing points
                int newPtIdx = this->find_valid_keypoints( keypoints, goodKeypointsBin, &unusedKeypoints, dist, neededFeatures);
                
                // insert the good points into the vector containing the new points of the whole image
                goodKeypoints.insert(goodKeypoints.end(), goodKeypointsBin.begin(), goodKeypointsBin.end());
                // save the unused keypoints for later
                if (newPtIdx < keypoints.size() - 1) {
                    unusedKeypoints.insert(unusedKeypoints.end(), keypoints.begin() + newPtIdx, keypoints.end());
                }
            }
        }
    }

    // The previous for loop found features in each bin. It might have found more features then requested. Delete from all bins for equal distancing
    if (goodKeypoints.size() > targetNumPoints) {
        int numFeaturesToRemove = goodKeypoints.size() - targetNumPoints;
        int stepSize = targetNumPoints / numFeaturesToRemove + 2;  // make sure the step size is big enough so we dont remove too many features

        std::vector<cv::KeyPoint> goodKeypoints_shortened;
        for (int i = 0; i < goodKeypoints.size(); i++) {
            if (i % stepSize) {
                goodKeypoints_shortened.push_back(goodKeypoints[i]);
            }
        }
        goodKeypoints = goodKeypoints_shortened;
    }

    // It also might have found less features then requested. Try to insert new points that were not used in the bins
    if (goodKeypoints.size() < targetNumPoints) {
        sort(unusedKeypoints.begin(), unusedKeypoints.end(), compare_keypoints);
        dist /= 2;  // reduce the distance criterion
        int newPtIdx = this->find_valid_keypoints( unusedKeypoints, goodKeypoints, NULL, dist, targetNumPoints);
    }

    if (goodKeypoints.empty()) {
        for (int i = 0; i < feature_status.size(); i++) {
            if (feature_status[i] == 2)
                feature_status[i] = 0;
        }
        return;
    }

    // Array to store the coordinates of the good keypoints in.
    std::vector<cv::Point2f> points;
   
    points.resize(goodKeypoints.size());
    for (int i = 0; i < goodKeypoints.size(); i++){
        points[i] = goodKeypoints[i].pt;
    }

    if (prev_corners.size() < feature_status.size())
        prev_corners.resize(feature_status.size());

    // Loop through each item in feature array, and insert new point if requested (status[i] == 2)
    int matches_idx = 0;
    for (int i = 0; i < feature_status.size(); i++) {
        if (feature_status[i] == FEATURE_STATUS_REQUEST) {
            if (matches_idx < points.size()) {
                // Insert the point in both prev and current features, as to not introduce wrong flows
                prev_corners[i] = points[matches_idx];
                prev_status[i] = FEATURE_STATUS_ACTIVE;

                features[i] = points[matches_idx];

                matches_idx++;
            } else {
                feature_status[i] = FEATURE_STATUS_INACTIVE;
            }
        }
    }
}