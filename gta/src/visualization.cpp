#include "include/visualization.h"


// void gta_ns::VisualizationNodelet::obj_cb(const gta::obj::ConstPtr &state_msg)
// {
    

//     obj_cal_pose.pose.position.x = state_msg->X_c;
//     obj_cal_pose.pose.position.y = state_msg->Y_c;
//     obj_cal_pose.pose.position.z = state_msg->Z_c;
//     // obj_gt_path.header.stamp = lidar->header.stamp;
//     obj_cal_path.poses.push_back(obj_cal_pose);
//     obj_cal_path.header.frame_id = "world";
//     pub_obj_cal.publish(obj_cal_path);
// }

void gta_ns::VisualizationNodelet::obj_syn_cb(const geometry_msgs::PoseStamped::ConstPtr &state_vicon, const gta::obj::ConstPtr &state_msg)
{
    pc_meas_pos[0] = state_msg->X_c;
    pc_meas_pos[1] = state_msg->Y_c;
    pc_meas_pos[2] = state_msg->Z_c;

    sensinfo.x = state_vicon->pose.position.x;
    sensinfo.y = state_vicon->pose.position.y;
    sensinfo.z = state_vicon->pose.position.z;
    sensinfo.ow = state_vicon->pose.orientation.w;
    sensinfo.ox = state_vicon->pose.orientation.x;
    sensinfo.oy = state_vicon->pose.orientation.y;
    sensinfo.oz = state_vicon->pose.orientation.z;

    Eigen::Matrix<double, 4, 1> obj_meas_body(pc_meas_pos[0], pc_meas_pos[1], pc_meas_pos[2], 1);

    Eigen::Matrix<double, 3, 3> matrix_for_q;
    Eigen::Quaterniond q2r_matrix(sensinfo.ow, sensinfo.ox, sensinfo.oy, sensinfo.oz);
    matrix_for_q = q2r_matrix.toRotationMatrix();

    Eigen::Matrix<double, 4, 4> body_to_world;
    body_to_world << matrix_for_q(0, 0), matrix_for_q(0, 1), matrix_for_q(0, 2), sensinfo.x,
        matrix_for_q(1, 0), matrix_for_q(1, 1), matrix_for_q(1, 2), sensinfo.y,
        matrix_for_q(2, 0), matrix_for_q(2, 1), matrix_for_q(2, 2), sensinfo.z,
        0, 0, 0, 1;

    Vector4d obj_meas_world = body_to_world * obj_meas_body;
    pc_meas_pos[0] = obj_meas_world[0];
    pc_meas_pos[1] = obj_meas_world[1];
    pc_meas_pos[2] = obj_meas_world[2];

    obj_cal_pose.pose.position.x = pc_meas_pos[0];
    obj_cal_pose.pose.position.y = pc_meas_pos[1];
    obj_cal_pose.pose.position.z = pc_meas_pos[2];
    // cout<<"obj_cal_pose: "<<obj_cal_pose.pose.position<<endl;

    // obj_cal_pose.header.frame_id = "world";
    obj_cal_pose.header.stamp = state_msg->header.stamp;
    pub_obj_body.publish(obj_cal_pose);

    // cout<<"state_msg->header.stamp: "<<state_msg->header.stamp<<endl;
    obj_cal_path.header.frame_id = "world";
    obj_cal_path.header.stamp = state_msg->header.stamp;
    obj_cal_path.poses.push_back(obj_cal_pose);
    pub_obj_cal.publish(obj_cal_path);
    // cout<<"frame_id: "<<obj_cal_path.header.frame_id<<endl;
    // cout<<"stamp: "<<obj_cal_path.header.stamp<<endl;
}

void gta_ns::VisualizationNodelet::gt_cb(const geometry_msgs::PoseStamped::ConstPtr &gt)
{
    // cout<<"gt->header.stamp: "<<gt->header.stamp<<endl;
    obj_gt_path.header.frame_id = "world";
    obj_gt_path.header.stamp = gt->header.stamp;
    obj_gt_pose.pose = gt->pose;
    obj_gt_path.poses.push_back(obj_gt_pose);
    pub_obj_gt.publish(obj_gt_path);
}


