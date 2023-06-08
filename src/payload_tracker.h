/**
 * payload_tracker.cpp
 *
 * Created on: June 5, 2023
 * Author: Juan de Dios
 */

#ifndef PAYLOAD_TRACKER_H_
#define PAYLOAD_TRACKER_H_

#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <vector>
#include <iostream>

using namespace cv;

class PayloadTracker 
{
public:
    PayloadTracker( void ); 
    void findQR(const cv::Mat &img);
    void init( int img_width, int img_height, int scale = 1);
    bool show( cv::Mat data, int scale = 1);
    bool show( cv::Mat data, cv::String &windowName,int scale = 1);
private:
    int image_height = 1080;
    int image_width = 1920;
    QRCodeDetector qrDet = QRCodeDetector();
    int scale; 
};

#endif