#include "include/To_odometry.h"

void gta_ns::OdomNodelet::obj_syn_cb(const nav_msgs::Odometry::ConstPtr &state_vicon, 
                                     const gta::obj::ConstPtr &state_msg)
{
    pc_meas_pos[0] = state_msg->X_c;
    pc_meas_pos[1] = state_msg->Y_c;
    pc_meas_pos[2] = state_msg->Z_c;

    sensinfo.x = state_vicon->pose.pose.position.x;
    sensinfo.y = state_vicon->pose.pose.position.y;
    sensinfo.z = state_vicon->pose.pose.position.z;
    sensinfo.ow = state_vicon->pose.pose.orientation.w;
    sensinfo.ox = state_vicon->pose.pose.orientation.x;
    sensinfo.oy = state_vicon->pose.pose.orientation.y;
    sensinfo.oz = state_vicon->pose.pose.orientation.z;

    // obj_gt_pose.pose.position.x = state_vicon->pose.pose.position.x;
    // obj_gt_pose.pose.position.y = state_vicon->pose.pose.position.y;
    // obj_gt_pose.pose.position.z = state_vicon->pose.pose.position.z;
    // obj_gt_path.header.frame_id = "world";
    // obj_gt_path.header.stamp = state_vicon->header.stamp;
    

    if(pc_meas_pos[0] == 0)
    {
        pc_meas_pos[0] = x_prev;
        pc_meas_pos[1] = y_prev;
        pc_meas_pos[2] = z_prev;
    }

    if(pc_meas_pos[0] != 0)
    {
       x_prev = pc_meas_pos[0];
       y_prev = pc_meas_pos[1];
       z_prev = pc_meas_pos[2];
    }

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

    obj_cal_pose.pose.position.x = obj_meas_world[0];
    obj_cal_pose.pose.position.y = obj_meas_world[1];
    obj_cal_pose.pose.position.z = obj_meas_world[2];
    cout<<"obj_cal_pose: "<<obj_cal_pose.pose.position<<endl;

    // obj_cal_pose.header.frame_id = "world";
    obj_cal_pose.header.stamp = state_msg->header.stamp;
    obj_pos_odm.publish(obj_cal_pose);
    // cout<<"state_msg->header.stamp: "<<state_msg->header.stamp<<endl;
    obj_cal_path.header.frame_id = "world";
    obj_cal_path.header.stamp = state_msg->header.stamp;
    obj_cal_path.poses.push_back(obj_cal_pose);
    pub_obj_cal.publish(obj_cal_path);
    // obj_gt_path.poses.push_back(obj_gt_pose);
    // pub_obj_gt.publish(obj_gt_path);
    cout<<"frame_id: "<<obj_cal_path.header.frame_id<<endl;
    cout<<"stamp: "<<obj_cal_path.header.stamp<<endl;
}


