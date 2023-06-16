#include "yoloqr.h"

void test(cv::Mat mymat) {
    /* Initialize network */
    //show(mymat, 1);
    Net net;     
    net = cv::dnn::readNetFromDarknet("external/opencv-yolo-qr-detection/qrcode-yolov3-tiny.cfg", "external/opencv-yolo-qr-detection/qrcode-yolov3-tiny.weights");
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    /* Convert mat to blob*/
    //cv::Mat mymat(512,512, CV_8UC3, Scalar(0,0,255));; 
    // cv::Scalar myscalar; 
    // cv::Size msize(416,416); cv::Size(416,416)
    cv::Mat blob =  cv::dnn::blobFromImage(mymat, 1.0f/255.0f, cv::Size(150,150), cv::Scalar(), true, false);
    // blob.reshape(blob.)
    // show(blob, 1);
    /* Insert blob to the network for inference */
    std::vector<std::string> layer_names = net.getLayerNames();
    std::vector<int> outLayers = net.getUnconnectedOutLayers();
    // std::cout << "unconnectedlayers " << outLayers.size() << std::endl;
    /*for(size_t i = 0; i < layer_names.size(); i++) 
        std::cout << layer_names[i] << std::endl;
    std::cout << std::endl;
    for(size_t i = 0; i < outLayers.size(); i++) 
        std::cout << outLayers[i] << std::endl;
    */    
    std::vector<std::string> names; 
    names.resize(outLayers.size());
    // std::cout << "layer names" << std::endl;
    for (size_t i = 0; i < outLayers.size() ; i++) {
        names[i] = layer_names[outLayers[i] - 1];
        // std::cout << names[i] << std::endl;
    } 
    net.setInput(blob);
    std::vector<std::vector<cv::Mat>> results; 
    std::cout << "Running net" << std::endl;
    net.forward(results, names);

    /* std::cout << "identified blobs, first vector size" << std::endl;
    std::cout << results.size() << std::endl;
    for (size_t i = 0; i < results.size(); i++) {
        std::cout << "identified blobs " << i << " second vector size" << std::endl;
        std::cout << results[i].size() << std::endl;
        for (size_t j = 0; j < results[i].size(); j++) {
              std::cout << results[i][j].size << std::endl;
        }
    } */

    /* Post postprocessing */
    std::vector<int> class_ids;
    std::vector<float> confidences; 
    std::vector<Rect> boxes;
    // Resizing factor. 
    int frameWidth = mymat.cols;
    int frameHeight = mymat.rows; 
    // std::cout << "Frame width and height " << frameWidth << " " << frameHeight << std::endl; 
    for(size_t i = 0; i < results.size(); i++) {
        std::vector<cv::Mat> result = results[i];
        for(size_t j = 0; j < result.size(); j++) {
            if(!result.empty()) {
                cv::Mat detection = result[j];
                // result.pop_back();
                int rowsNoOfDetection = detection.rows;
                // The coluns looks like this, The first is region center x, center y, width
                // height, The class 1 - N is the column entries, which gives you a number, 
                // where the biggist one corrsponding to most probable class. 
                // [x ; y ; w; h; class 1 ; class 2 ; class 3 ;  ; ;....]
                //  
                int colsCoordinatesPlusClassScore = detection.cols;
                // Loop over number of detected object. 
                // std::cout << j << " Confidence ";
                for (int k = 0; k < rowsNoOfDetection; ++k) {
                    // for each row, the score is from element 5 up
                    // to number of classes index (5 - N columns)
                    Mat scores = detection.row(k).colRange(5, colsCoordinatesPlusClassScore);
                    // std::cout << detection.at<float>(k,5);
                    Point PositionOfMax;
                    double confidence;

                    // This function find indexes of min and max confidence and related index of element. 
                    // The actual index is match to the concrete class of the object.
                    // First parameter is Mat which is row [5fth - END] scores,
                    // Second parameter will gives you min value of the scores. NOT needed 
                    // confidence gives you a max value of the scores. This is needed, 
                    // Third parameter is index of minimal element in scores
                    // the last is position of the maximum value.. This is the class!!
                    minMaxLoc(scores, 0, &confidence, 0, &PositionOfMax);
                    if (confidence > 0.6) {
                        std::cout << " confi " << confidence << std::endl; 
                    /*if (confidence > 0.0001) {*/
                        // thease four lines are
                        // [x ; y ; w; h;
                        int centerX = (int)(detection.at<float>(k, 0) * mymat.cols); 
                        int centerY = (int)(detection.at<float>(k, 1) * mymat.rows); 
                        int width =   (int)(detection.at<float>(k, 2) * mymat.cols+20); 
                        int height =   (int)(detection.at<float>(k, 3) * mymat.rows+100); 

                        int left = centerX - width / 2;
                        int top = centerY - height / 2;


                        stringstream ss;
                        ss << PositionOfMax.x;
                        string clas = ss.str();
                        int color = PositionOfMax.x * 10;
                        putText(mymat, clas, Point(left, top), 1, 2, Scalar(color, 255, 255), 2, false);
                        stringstream ss2;
                        ss << confidence;
                        string conf = ss.str();

                        rectangle(mymat, Rect(left, top, width, height), Scalar(color, 0, 0), 2, 8, 0);
                        show(mymat, 1);
                    }
                }
                // std::cout << std::endl; 

            }
        }
    }
}

void test(){}

bool show(cv::Mat data, int scale)
{
    static int frame_number = 0;
    cv::Mat temp;
    char buffer[20];

    //if (scale != 1)
    //    cv::resize(data, temp, cv::Size(image_width / scale, image_height / scale));
    //else
        temp = data.clone();

    // Show image using opencv image container
    cv::imshow("justforshow", temp);

    // Listen for any key-presses
    int key = cv::waitKey(10);

    // Taking a snapshot and saving it to the device
    if (key == 99)
    { // 'c' for capture
        sprintf(buffer, "images/dnnframeP-%d.jpg", frame_number);
        cv::imwrite(buffer, data);
        frame_number++;
    }
    // Return false, to exit application
    else if (key == 27) // 'esc' to end
        return false;

    return true;
}