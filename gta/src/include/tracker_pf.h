#ifndef TRACKER_PF_H
#define TRACKER_PF_H

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

#include <boost/format.hpp>

#include <mutex>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gta/obj.h"

using namespace Eigen;
using namespace std;
using namespace pcl::tracking;
using namespace std::chrono_literals;

typedef pcl::PointXYZRGBA RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

namespace gta_ns
{
    CloudPtr transed_ref(new Cloud);
    CloudPtr transed_ref_downsampled(new Cloud);
    CloudPtr cloud_pass_(new Cloud);
    CloudPtr cloud_pass_downsampled_(new Cloud);
    CloudPtr target_cloud(new Cloud);

    ParticleFilter::Ptr tracker_(new ParticleFilter);

    class TrackerNodeletClass : public nodelet::Nodelet
    {
    public:
        TrackerNodeletClass() { ; }
        ~TrackerNodeletClass() { ; }

    private:
        // Subscriber
        ros::Subscriber sub_pcl;
        ros::Subscriber sub_lidar;

        // Publisher
        ros::Publisher pub_obj_body;

        //ros msg
        gta::obj pose_b_msg;

        // callback
        void pc_cb(const gta::obj::ConstPtr &pc);
        void lidar_cb(const livox_ros_driver::CustomMsg::ConstPtr &lidar);

        void filterPassThrough (const CloudConstPtr &cloud, Cloud &result);//Filter along a specified dimension
        void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size);
        bool drawParticles (pcl::visualization::PCLVisualizer& viz);//Draw the current particles
        void drawResult (pcl::visualization::PCLVisualizer& viz);//Draw model reference point cloud
        void viz_cb (pcl::visualization::PCLVisualizer& viz);//visualization's callback function
        void run_track (const CloudConstPtr &cloud);//call pcl::track


        std::mutex mtx_;
        bool new_cloud_;
        bool yolo_in = false;
        bool init = false;
        bool ref_in = false;
        double downsampling_grid_size_;
        Vector3f pc_int_pos;
        Vector3f obj_pos_b;
        int counter = 0;
        Eigen::Affine3f result;

        virtual void onInit()
        {

            ROS_INFO("--------tracker node has been created---------");
            ros::NodeHandle nh = getPrivateNodeHandle();

            sub_pcl = nh.subscribe<gta::obj>("/pc_input", 1,
                                                        &TrackerNodeletClass::pc_cb,this);
            sub_lidar = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10000,
                                                        &TrackerNodeletClass::lidar_cb,this);

            pub_obj_body = nh.advertise<gta::obj>("/obj_pos_body", 10);
        }
    };
}

// PLUGINLIB_EXPORT_CLASS(gta_ns::TrackerNodeletClass, nodelet::Nodelet)

#endif