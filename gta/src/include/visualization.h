#ifndef VISUALIZATION_H
#define VISUALIZATION_H

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

#include "yamlRead.h"
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


    class VisualizationNodelet : public nodelet::Nodelet
    {
    public:
        VisualizationNodelet() { ; }
        ~VisualizationNodelet() { ; }

    private:
        // Subscriber
        ros::Subscriber sub_gt;
        ros::Subscriber sub_obj;

        //Synchronized Subscriber
        message_filters::Subscriber<geometry_msgs::PoseStamped> sub_sensors_syn;
        message_filters::Subscriber<gta::obj> sub_obj_syn;
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
        // void obj_cb(const gta::obj::ConstPtr &state_msg);
        void obj_syn_cb(const geometry_msgs::PoseStamped::ConstPtr &state_vicon, const gta::obj::ConstPtr &state_msg);
        void gt_cb(const geometry_msgs::PoseStamped::ConstPtr &gt);


        bool yolo_in = false;
        bool init = false;
        Vector3f pc_meas_pos;
        Vector3f obj_pos_w;
        int counter = 0;
        Senspose sensinfo;
        Mat4x4 mat_cam_lidar;


        virtual void onInit()
        {
            ROS_INFO("--------visualization node has been created---------");
            ros::NodeHandle nh = getMTPrivateNodeHandle();
            
            string configFilePath;
            nh.getParam("yamlconfigfile", configFilePath);
            mat_cam_lidar = Mat44FromYaml(configFilePath, "T_cam_lidar");
            cout<<mat_cam_lidar<<endl;
            cout << configFilePath << endl;



            sub_gt = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10,
                                                        &VisualizationNodelet::gt_cb,this);
            // sub_obj = nh.subscribe<gta::obj>("/state_input", 10000,
            //                                             &VisualizationNodelet::obj_cb,this);
            sub_sensors_syn.subscribe(nh,"/vrpn_client_node/gh034_sensors_lhj/pose",1);
            sub_obj_syn.subscribe(nh,"/state_input",1);
            sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), sub_sensors_syn, sub_obj_syn);
            sync->registerCallback(boost::bind(&VisualizationNodelet::obj_syn_cb, this, _1, _2));


            pub_obj_gt = nh.advertise<nav_msgs::Path>("/obj_gt_path", 1);
            pub_obj_cal = nh.advertise<nav_msgs::Path>("/obj_cal_path", 1);
            pub_obj_body = nh.advertise<geometry_msgs::PoseStamped>("/obj_pos_w",1);

            
        }
    };
}

PLUGINLIB_EXPORT_CLASS(gta_ns::VisualizationNodelet, nodelet::Nodelet)

#endif