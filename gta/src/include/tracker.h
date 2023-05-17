#ifndef TRACKER_H
#define TRACKER_H

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Path.h>
#include <livox_ros_driver/CustomMsg.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gta/obj.h"

using namespace Eigen;
using namespace std;

namespace gta_ns
{
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_pc(new pcl::PointCloud<pcl::PointXYZ>);

    int stateSize = 6;
    int measSize = 3;
    int contrSize = 0;
    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z] need z as well
    cv::Mat meas(measSize, 1, type);    // [x,y,z]


    class TrackerNodeletClass : public nodelet::Nodelet
    {
    public:
        TrackerNodeletClass() { ; }
        ~TrackerNodeletClass() { ; }

    private:
        // Subscriber
        ros::Subscriber sub_gt;
        ros::Subscriber sub_obj;

        //Synchronized Subscriber
        message_filters::Subscriber<geometry_msgs::PoseStamped> sub_sensor_syn;
        message_filters::Subscriber<gta::obj> sub_pc_syn;
        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, gta::obj> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> * sync;

        // Publisher
        ros::Publisher pub_obj_body;
        ros::Publisher pub_obj_gt, pub_obj_cal;               // rviz path pubilisher

        //ros msg
        gta::obj pose_b_msg;
        nav_msgs::Path obj_gt_path, obj_cal_path;             // rviz Path msg
        geometry_msgs::PoseStamped obj_gt_pose, obj_cal_pose; // object PoseStamped msg


        // callback
        void obj_cb(const gta::obj::ConstPtr &pc);
        // void obj_cb(const gta::obj::ConstPtr &obj);
        void gt_cb(const geometry_msgs::PoseStamped::ConstPtr &gt);


        bool yolo_in = false;
        bool init = false;
        Vector3f pc_meas_pos;
        Vector3f obj_pos_w;
        int counter = 0;
        Senspose sensinfo;

        //kf variable
        double dT;
        double ticks = 0;
        Vector3f state_pre;
        Vector3f state_true;
        int notFoundCount;

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

            ROS_INFO("--------tracker node has been created---------");
            ros::NodeHandle nh = getPrivateNodeHandle();
            sub_gt = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_ball_lhj/pose", 10000,
                                                        &TrackerNodeletClass::gt_cb,this);
            sub_obj = nh.subscribe<gta::obj>("/pc_input", 10000,
                                                        &TrackerNodeletClass::obj_cb,this);
            // sub_sensor_syn.subscribe(nh,"/vrpn_client_node/gh034_sensor_lhj/pose",1000);
            // sub_pc_syn.subscribe(nh,"/pc_input",10000);
            // sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10000), sub_sensor_syn, sub_pc_syn);
            // sync->registerCallback(boost::bind(&TrackerNodeletClass::obj_cb, this, _1, _2));


            pub_obj_body = nh.advertise<gta::obj>("/obj_pos_body", 10);
            pub_obj_gt = nh.advertise<nav_msgs::Path>("/obj_gt_path", 1);
            pub_obj_cal = nh.advertise<nav_msgs::Path>("/obj_cal_path", 1);

            
        }
    };
}

PLUGINLIB_EXPORT_CLASS(gta_ns::TrackerNodeletClass, nodelet::Nodelet)

#endif