#include "payload_tracker.h"

using namespace cv;

PayloadTracker::PayloadTracker(void) {}
void PayloadTracker::init( int img_width, int img_height, int scale )
{
    image_width = img_width;
    image_height = img_height;
}

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
            cv::rotate(img, img, ROTATE_180);
            /* Rotate the point text to put, 180 degrees */
            cv::Point text_position(img.cols - pointsQR.at<int>(0,0), img.rows - pointsQR.at<int>(0,1));
            cv::putText(img, data, text_position, cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv::LINE_AA);
            std::cout << "Points " << pointsQR.rows << " " << pointsQR.cols << std::endl;
            for ( int i = 0; i < (pointsQR.rows + 1) * pointsQR.cols; i++) {
                std::cout << "coord row " << 0 << " col " << i << " data " << pointsQR.at<int>(0,i) << std::endl; 
            }
            std::cout << pointsQR <<std::endl;
            /* Calculate angle, points are in a clockwise direction, rotate 180 degrees, remember image angle coordinates*/
            double p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y; 
            p1x =   img.cols - pointsQR.at<int>(0,0);
            p1y = -(img.rows - pointsQR.at<int>(0,1));
            p2x =   img.cols - pointsQR.at<int>(0,2);
            p2y = -(img.rows - pointsQR.at<int>(0,3));
            p3x =   img.cols - pointsQR.at<int>(0,4);
            p3y = -(img.rows - pointsQR.at<int>(0,5));
            p4x =   img.cols - pointsQR.at<int>(0,6);
            p4y = -(img.rows - pointsQR.at<int>(0,7));
            double angle = atan2(p1y - p2y, p1x- p2x);
            double angledeg = angle * 180 / 3.141592; 
            std::cout << "angle 1 " << angledeg << std::endl;
            angle = atan2(p3y - p4y, p3x- p4x);
            angledeg = angle * 180 / 3.141592; 
            std::cout << "angle 2 " << angledeg << std::endl;
            angle = atan2(p2y - p3y, p2x- p3x);
            angledeg = angle * 180 / 3.141592; 
            std::cout << "angle 3 " << angledeg << std::endl;
            angle = atan2(p4y - p1y, p4x- p1x);
            angledeg = angle * 180 / 3.141592; 
            std::cout << "angle 4 " << angledeg << std::endl;
            cv::circle( img, cv::Point(p1x, -p1y), 2, cv::Scalar(255, 255, 255), -1 );
            cv::putText(img, "P1", cv::Point(p1x, -p1y), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv::LINE_AA);
            cv::circle( img, cv::Point(p2x, -p2y), 2, cv::Scalar(255, 255, 255), -1 );
            cv::putText(img, "P2", cv::Point(p2x, -p2y), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv::LINE_AA);
            cv::circle( img, cv::Point(p3x, -p3y), 2, cv::Scalar(255, 255, 255), -1 );
            cv::putText(img, "P3", cv::Point(p3x, -p3y), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv::LINE_AA);
            cv::circle( img, cv::Point(p4x, -p4y), 2, cv::Scalar(255, 255, 255), -1 );
            cv::putText(img, "P4", cv::Point(p4x, -p4y), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv::LINE_AA);

            cv::putText(img, std::to_string(angledeg), cv::Point((p2x + p4x)/2,(-p2y -p4y)/2), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv::LINE_AA);

                        
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