#ifndef YOLO_H
#define YOLO_H

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <sstream>

#include <std_msgs/Bool.h>
#include <cmath>
#include <numeric>
#include <eigen3/Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include "/home/luo/gtanodelet/src/GTA/gta/include/ncnn/net.h"
#include "gta/vertice.h"

using namespace std;

typedef struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob = 0; // confidence
    string classnameofdetection;
    cv::Mat frame;
    cv::Point center_bdbox;
} Object;

static const char *class_names[] = {"null", "ball", "uav"};

class YoloNode
{
public:
    YoloNode() { ; }
    ~YoloNode() { ; }

private:
    float set_confidence;

    chrono::time_point<chrono::steady_clock> total_start, total_end, dnn_start, dnn_end;
    float total_fps;

    bool intiated = false;

    // init parameters
    string param;
    string bin;
    int target_size;

    ncnn::Net *cnn_local = new ncnn::Net();

    float appro_fps;
    std::vector<Object> objects;
    cv::Mat frame, res, gt;

    // Subscriber
    ros::Subscriber subimage;
    // Publisher
    ros::Publisher pub_ver;

    void YoloNodeletClassInit(const char *parampath_input, const char *binfile_input);
    void detect_yolo(cv::Mat &bgr);
    void draw_objects(cv::Mat &bgr, const std::vector<Object> &objects); // void display(cv::Mat frame);

    // callback
    void img_cb(const sensor_msgs::CompressedImage::ConstPtr &rgbimage);
};

#endif