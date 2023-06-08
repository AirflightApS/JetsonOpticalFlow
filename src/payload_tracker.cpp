#include "payload_tracker.h"

using namespace cv;

PayloadTracker::PayloadTracker(void) {}

void PayloadTracker::findQR(const cv::Mat &img)
{
    if (!img.data)
        throw "Image is invalid";

    Mat points;
    // bool detection_result = qrDet.detect(img, points);
    // std::cout << "detection_result: " << detection_result << std::endl;

    Mat corners, rectImage;
    try{
        std::string data = qrDet.detectAndDecodeCurved(img, points, rectImage); // <- Throws an error on flood and fill 
        if ( data.length() > 0)
        {
            std::cout << "QR detected " /*<< data*/ << std::endl;
            /* Draw the surrounding box if found */
            cv::Mat pointsQR; 
            points.copyTo(pointsQR);
            pointsQR.convertTo(pointsQR, CV_32S);
            std::cout << "Convertion ended " /*<< data*/ << std::endl;
            
            cv::polylines(img, pointsQR, true, (0, 255, 0), 3);
            std::cout << "Points " << pointsQR.rows << " " << pointsQR.cols << std::endl;

            show(img);
            cv::String winname = "QRcode";
            show(rectImage, winname);
            // cv::line(image, cv::Point(features_previous[i].x, features_previous[i].y), cv::Point(features_current[i].x, features_current[i].y), cv::Scalar(255, 255, 255));
            // cv::circle(image, cv::Point(features_current[i].x, features_current[i].y), 2, cv::Scalar(255, 255, 255), -1);
        }
        else
        {
            std::cout << "QR Code not detected" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        
    }
}

bool PayloadTracker::show(cv::Mat data, int scale)
{
    static int frame_number = 0;
    cv::Mat temp = cv::Mat(image_height, image_width, CV_8UC1);
    char buffer[20];

    if (scale != 1)
        cv::resize(data, temp, cv::Size(image_width / scale, image_height / scale));
    else
        temp = data.clone();

    // Show image using opencv image container
    cv::imshow("Payload", temp);

    // Listen for any key-presses
    int key = cv::waitKey(10);

    // Taking a snapshot and saving it to the device
    if (key == 99)
    { // 'c' for capture
        sprintf(buffer, "images/frameP-%d.jpg", frame_number);
        cv::imwrite(buffer, data);
        frame_number++;
    }
    // Return false, to exit application
    else if (key == 27) // 'esc' to end
        return false;

    return true;
}

bool PayloadTracker::show(cv::Mat data, cv::String &windowName, int scale)
{
    static int frame_number = 0;
    cv::Mat temp = cv::Mat(image_height, image_width, CV_8UC1);
    char buffer[20];

    if (scale != 1)
        cv::resize(data, temp, cv::Size(image_width / scale, image_height / scale));
    else
        temp = data.clone();

    // Show image using opencv image container
    cv::imshow(windowName, temp);

    // Listen for any key-presses
    int key = cv::waitKey(10);

    // Taking a snapshot and saving it to the device
    if (key == 99)
    { // 'c' for capture
        sprintf(buffer, "images/framer-%d.jpg", frame_number);
        cv::imwrite(buffer, data);
        frame_number++;
    }
    // Return false, to exit application
    else if (key == 27) // 'esc' to end
        return false;

    return true;
}