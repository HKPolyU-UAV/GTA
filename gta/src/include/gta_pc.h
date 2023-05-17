#ifndef GTA_PC_H
#define GTA_PC_H

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"


#include <std_msgs/Bool.h>
#include <cmath>
#include <numeric>
#include <eigen3/Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Path.h>
#include <livox_ros_driver/CustomMsg.h>

#include "/home/luo/gtanodelet/src/GTA/gta/include/ncnn/net.h"
#include "gta/vertice.h"
#include "yamlRead.h"
#include "gta/obj.h"

#include <sstream>

using namespace std;
using namespace Eigen;



namespace gta_ns{

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    ros::Rate rate(1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_pc(new pcl::PointCloud<pcl::PointXYZ>);

    int stateSize = 6;
    int measSize = 3;
    int contrSize = 0;
    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z] need z as well
    cv::Mat meas(measSize, 1, type);    // [x,y,z]

    static const char *class_names[] = {"null", "ball", "uav"};

    typedef struct Object
    {
        cv::Rect_<float> rect;
        int label;
        float prob = 0; // confidence
        string classnameofdetection;
        cv::Mat frame;
        cv::Point center_bdbox;
    } Object;

    typedef struct Senspose
    {
        double x;
        double y;
        double z;
        double ow;
        double ox;
        double oy;
        double oz;
    } Senspose;

    class TrackingNodelet : public nodelet::Nodelet
    {
        public:
        TrackingNodelet(){;}
        ~TrackingNodelet(){;}

        private:

        //yoloDepthnode
            Vector3f img_meas_pos;
            float set_confidence;
            chrono::time_point<chrono::steady_clock> total_start, total_end, dnn_start, dnn_end;
            float total_fps;
            bool intiated = false;
            bool yolo_in = false;
            // init parameters
            string param;
            string bin;
            int target_size;
            ncnn::Net *cnn_local = new ncnn::Net();
            float appro_fps;
            std::vector<Object> objects;
            cv::Mat frame, res, gt;
            // Synchronized Subscriber
            message_filters::Subscriber<sensor_msgs::CompressedImage> sub_img_syn;
            message_filters::Subscriber<sensor_msgs::Image> sub_dp_syn;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            message_filters::Synchronizer<MySyncPolicy> *sync;
            // Publisher
            ros::Publisher pub_ver;
            void YoloNodeInit(const char *parampath_input, const char *binfile_input);
            void detect_yolo(cv::Mat &bgr);
            void draw_objects(cv::Mat &bgr, const std::vector<Object> &objects); // void display(cv::Mat frame);
            // callback
            void imgsyn_cb(const sensor_msgs::CompressedImage::ConstPtr &rgbimage, const sensor_msgs::ImageConstPtr &dpimage);
        //yoloDepthnode

        // depth assignment
            // kf variable
            double dT;
            double ticks = 0;
            Vector3f state_pre;
            Vector3f state_true;
            Vector3f meas_pos;
            int notFoundCount;
            // point cloud process variable
            Vector3f pc_meas_pos;
            bool use_dpeth = false;
            bool kf_init = false;
            cv::Point depthbox_vertice1;
            cv::Point depthbox_vertice2;
            Mat4x4 mat_lidar_cam;
            Mat3x4 P_cam;
            Mat3x3 K_cam;
            cv::Point center_pixel;
            // obj_pose_cam msg
            sensor_msgs::PointCloud2 obj_pc_msg;
            gta::obj obj_pos_pc;
            // Subscriber
            ros::Subscriber subpcl;
            ros::Subscriber subdepth;
            ros::Subscriber substate;
            // Publisher
            ros::Publisher pub_meas;
            // callback
            void pcl_cb(const livox_ros_driver::CustomMsg::ConstPtr &pclmsg);
            // void dp_cb(const sensor_msgs::ImageConstPtr &dpmsg);
            // void state_cb(const gta::obj::ConstPtr &statemsg);
            //func
            void PTfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_track, double filter_size);
            void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
        // depth assignment


            virtual void onInit()
            {
                // set kalman filter parameter
                cv::setIdentity(kf.transitionMatrix);

                kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
                kf.measurementMatrix.at<float>(0) = 1.0f;
                kf.measurementMatrix.at<float>(7) = 1.0f;
                kf.measurementMatrix.at<float>(14) = 1.0f;

                kf.processNoiseCov.at<float>(0) = 1e-2;
                kf.processNoiseCov.at<float>(7) = 1e-2;
                kf.processNoiseCov.at<float>(14) = 1e-2;
                kf.processNoiseCov.at<float>(21) = 5.0f;
                kf.processNoiseCov.at<float>(28) = 5.0f;
                kf.processNoiseCov.at<float>(35) = 5.0f;

                cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

                ROS_INFO("--------tracking node has been created---------");
                ros::NodeHandle nh = getMTPrivateNodeHandle();

                // initialize
                string configFilePath;

                nh.getParam("parampath", param);
                nh.getParam("binpath", bin);
                nh.getParam("target_size", target_size);
                nh.getParam("yamlconfigfile", configFilePath);

                // Extrinsic parameters from lidar to camera
                mat_lidar_cam = Mat44FromYaml(configFilePath, "T_lidar_cam");
                P_cam = Mat34FromYaml(configFilePath, "P_cam");
                K_cam = Mat33FromYaml(configFilePath, "K_cam");
                cout<<mat_lidar_cam<<endl;
                cout<<P_cam<<endl;
                cout<<K_cam<<endl;

                char *parampath = (char *)param.data();
                char *binpath = (char *)bin.data();
                YoloNodeInit(parampath, binpath);
                cout << param << endl;
                cout << bin << endl;
                cout << configFilePath << endl;

                // subscribe
                // subimage = nh.subscribe<sensor_msgs::CompressedImage>("/camera/color/image_raw/compressed", 10,
                //                                                       &TrackingNodelet::img_cb, this);
                sub_img_syn.subscribe(nh, "/camera/color/image_raw/compressed", 10);
                sub_dp_syn.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 10000);
                sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10000), sub_img_syn, sub_dp_syn);
                sync->registerCallback(boost::bind(&TrackingNodelet::imgsyn_cb, this, _1, _2));

                subpcl = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10,
                                                                      &TrackingNodelet::pcl_cb, this);
                // subdepth = nh.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 10,
                //                                                       &TrackingNodelet::dp_cb, this);
                // substate= nh.subscribe<gta::obj>("/obj_pos_body", 10,
                //                                                       &TrackingNodelet::state_cb, this);

                // publisher
                pub_ver = nh.advertise<gta::vertice>("/obj_box_vertices", 1);
                pub_meas = nh.advertise<gta::obj>("/obj_pos_body", 1);
        }
    };



}




PLUGINLIB_EXPORT_CLASS(gta_ns::TrackingNodelet, nodelet::Nodelet)
#endif