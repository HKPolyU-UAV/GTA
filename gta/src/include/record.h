#ifndef RECORD_H
#define RECORD_H
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Bool.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

static double err_x, err_y, err_z;

using namespace std;

namespace gta_ns
{
    class RecordNodelet : public nodelet::Nodelet
    {
    public:
        RecordNodelet() { ; }
        ~RecordNodelet() { ; }

    private:
        //Synchronized Subscriber
        message_filters::Subscriber<geometry_msgs::PoseStamped> obj_info_sub;
        message_filters::Subscriber<geometry_msgs::PoseStamped> sub_obj_w;
        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> * sync;

        // callback
        // void obj_cb(const gta::obj::ConstPtr &state_msg);
        void err_cal(const geometry_msgs::PoseStamped::ConstPtr &obj_pose_w, const geometry_msgs::PoseStamped::ConstPtr &obj_pose_gt);


        virtual void onInit()
        {
            ROS_INFO("--------record node has been created---------");
            ros::NodeHandle nh = getMTPrivateNodeHandle();
            
            obj_info_sub.subscribe(nh,"/mavros/vision_pose/pose",1);
            sub_obj_w.subscribe(nh,"/obj_pos_w",1);
            sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), obj_info_sub, sub_obj_w);
            sync->registerCallback(boost::bind(&RecordNodelet::err_cal, this, _1, _2));
        }
    };
}

PLUGINLIB_EXPORT_CLASS(gta_ns::RecordNodelet, nodelet::Nodelet)

#endif