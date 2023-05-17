#ifndef TO_ODOMETRY_H
#define TO_ODOMETRY_H

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

#include "nav_msgs/Odometry.h"

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

    static double obj_x_c, obj_y_c, obj_z_c;
    static double x_prev = 0, y_prev = 0, z_prev = 0; // previous obj_camera;
    Eigen::Matrix<double, 4, 1> obj_odm;
    

    class OdomNodelet : public nodelet::Nodelet
    {
    public:
        OdomNodelet() { ; }
        ~OdomNodelet() { ; }

    private:
        // Subscriber
        ros::Subscriber sub_gt;
        ros::Subscriber sub_obj;

        //Synchronized Subscriber
        message_filters::Subscriber<nav_msgs::Odometry> sub_lidar_odm;
        message_filters::Subscriber<gta::obj> sub_obj_syn;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, gta::obj> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> * sync;

        // Publisher
        ros::Publisher pub_obj_body;
        ros::Publisher pub_obj_gt, pub_obj_cal;               // rviz path pubilisher
        ros::Publisher obj_pos_odm;

        //ros msg
        gta::obj pose_b_msg;
        nav_msgs::Path obj_gt_path, obj_cal_path;             // rviz Path msg
        geometry_msgs::PoseStamped obj_gt_pose, obj_cal_pose; // object PoseStamped msg


        // callback
        // void obj_cb(const gta::obj::ConstPtr &state_msg);
        void obj_syn_cb(const nav_msgs::Odometry::ConstPtr &state_vicon, 
                        const gta::obj::ConstPtr &state_msg);


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


            sub_lidar_odm.subscribe(nh,"/Odometry",1);
            sub_obj_syn.subscribe(nh,"/state_input",1);
            sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), sub_lidar_odm, sub_obj_syn);
            sync->registerCallback(boost::bind(&OdomNodelet::obj_syn_cb, this, _1, _2));


            pub_obj_gt = nh.advertise<nav_msgs::Path>("/obj_gt_path", 1);
            pub_obj_cal = nh.advertise<nav_msgs::Path>("/obj_cal_path", 1);

            obj_pos_odm = nh.advertise<geometry_msgs::PoseStamped> ("/scout_wp/pose",1);

        }
    };
}

PLUGINLIB_EXPORT_CLASS(gta_ns::OdomNodelet, nodelet::Nodelet)

#endif