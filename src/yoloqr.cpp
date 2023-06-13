#include "yoloqr.h"

void test(cv::Mat mymat) {
    /* Initialize network */
    Net net;     
    net = cv::dnn::readNetFromDarknet("external/opencv-yolo-qr-detection/qrcode-yolov3-tiny.cfg", "external/opencv-yolo-qr-detection/qrcode-yolov3-tiny.weights");
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    /* Convert mat to blob*/
    //cv::Mat mymat(512,512, CV_8UC3, Scalar(0,0,255));; 
    // cv::Scalar myscalar; 
    // cv::Size msize(416,416);
    cv::Mat blob =  cv::dnn::blobFromImage(mymat, 1.0f/255.0f, cv::Size(416,416), cv::Scalar(), true, false);
    /* Insert blob to the network for inference */
    std::vector<std::string> layer_names = net.getLayerNames();
    std::vector<int> outLayers = net.getUnconnectedOutLayers();
    std::vector<std::string> names; 
    names.resize(outLayers.size());
    std::cout << "layer names" << std::endl;
    for (size_t i = 0; i < outLayers.size(); i++) {
        names[i] = layer_names[outLayers[i] - 1];
        std::cout << names[i] << std::endl;
    }
    net.setInput(blob);
    std::vector<std::vector<cv::Mat>> results; 
    net.forward(results, names);

    std::cout << "identified blobs, first vector size" << std::endl;
    std::cout << results.size() << std::endl;
    for (size_t i = 0; i < results.size(); i++) {
        std::cout << "identified blobs, second vector size" << std::endl;
        std::cout << results[i].size() << std::endl;
        for (size_t j = 0; j < results[i].size(); j++) {
              std::cout << results[i][j].size << std::endl;
        }
    }
}

void test(){}