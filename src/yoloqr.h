/**
 * payload_tracker.cpp
 *
 * Created on: June 5, 2023
 * Author: Juan de Dios
 */

#ifndef YOLOQR_H_
#define YOLOQR_H_

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
#include <fstream>

using namespace cv;
using namespace std;
using namespace cv::dnn;

void test(cv::Mat mymat);
void test();
bool show(cv::Mat data, int scale);
#endif