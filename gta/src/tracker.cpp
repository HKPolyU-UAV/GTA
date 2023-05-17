#include "include/tracker.h"


void gta_ns::TrackerNodeletClass::obj_cb(const gta::obj::ConstPtr &pc_msg)
{
    // pcl::fromROSMsg(pc_msg->pointcloud, *obj_pc);
    pc_meas_pos[0] = pc_msg->X_c;
    pc_meas_pos[1] = pc_msg->Y_c;
    pc_meas_pos[2] = pc_msg->Z_c;
    // cout << "obj_pc->points.size(): " << obj_pc->points.size() << endl;

    // sensinfo.x = vrpn_sens->pose.position.x;
    // sensinfo.y = vrpn_sens->pose.position.y;
    // sensinfo.z = vrpn_sens->pose.position.z;
    // sensinfo.ow = vrpn_sens->pose.orientation.w;
    // sensinfo.ox = vrpn_sens->pose.orientation.x;
    // sensinfo.oy = vrpn_sens->pose.orientation.y;
    // sensinfo.oz = vrpn_sens->pose.orientation.z;

    // Eigen::Matrix<double, 4, 1> obj_meas_body(pc_meas_pos[0], pc_meas_pos[1], pc_meas_pos[2], 1);

    // Eigen::Matrix<double, 3, 3> matrix_for_q;
    // Eigen::Quaterniond q2r_matrix(sensinfo.ow, sensinfo.ox, sensinfo.oy, sensinfo.oz);
    // matrix_for_q = q2r_matrix.toRotationMatrix();

    // Eigen::Matrix<double, 4, 4> body_to_world;
    // body_to_world << matrix_for_q(0, 0), matrix_for_q(0, 1), matrix_for_q(0, 2), sensinfo.x,
    //     matrix_for_q(1, 0), matrix_for_q(1, 1), matrix_for_q(1, 2), sensinfo.y,
    //     matrix_for_q(2, 0), matrix_for_q(2, 1), matrix_for_q(2, 2), sensinfo.z,
    //     0, 0, 0, 1;

    // Vector4d obj_meas_world = body_to_world * obj_meas_body;
    // pc_meas_pos[0] = obj_meas_world[0];
    // pc_meas_pos[1] = obj_meas_world[1];
    // pc_meas_pos[2] = obj_meas_world[2];

    if (obj_pc->points.size() > 0)
    {

        yolo_in = true;
    }
    else
    {
        yolo_in = false;
    }
    cout << "yolo_in: " << yolo_in << endl;

    //run kalman filter
    if (!init)
    {
        // >>>> Initialization
        kf.errorCovPre.at<float>(0) = 1; // px
        kf.errorCovPre.at<float>(7) = 1; // px
        kf.errorCovPre.at<float>(14) = 1;
        kf.errorCovPre.at<float>(21) = 1;
        kf.errorCovPre.at<float>(28) = 1; // px
        kf.errorCovPre.at<float>(35) = 1; // px

        state.at<float>(0) = meas.at<float>(0);
        state.at<float>(1) = meas.at<float>(1);
        state.at<float>(2) = meas.at<float>(2);
        state.at<float>(3) = 0;
        state.at<float>(4) = 0;
        state.at<float>(5) = 0;
        // <<<< Initialization

        kf.statePost = state;
        init = true;
    }

    double precTick = ticks;
    ticks = (double)cv::getTickCount();
    dT = (ticks - precTick) / cv::getTickFrequency(); // seconds
    // >>>> Matrix A
    kf.transitionMatrix.at<float>(3) = dT;
    kf.transitionMatrix.at<float>(10) = dT;
    kf.transitionMatrix.at<float>(17) = dT;

    state = kf.predict();

    state_pre[0] = state.at<float>(0);
    state_pre[1] = state.at<float>(1);
    state_pre[2] = state.at<float>(2);

    if(yolo_in)
    {

        notFoundCount = 0;

        meas.at<float>(0) = pc_meas_pos[0];
        meas.at<float>(1) = pc_meas_pos[1];
        meas.at<float>(2) = pc_meas_pos[2];

        kf.correct(meas); // Kalman Correction

        state_true[0] = state.at<float>(0);
        state_true[1] = state.at<float>(1);
        state_true[2] = state.at<float>(2);

    }

    if (yolo_in)
    {
        cout << "show measure: " << endl;
        // send.header.stamp = ros::Time::now();
        obj_pos_w = state_true;
        cout << "object position: \n" << state_true << endl;
    }
    else
    {
        cout << "show predict" << endl;
        // send.header.stamp = ros::Time::now();
        obj_pos_w = state_pre;
        cout << "object position: \n" << state_pre << endl
             << endl;
    }
    pose_b_msg.X_c = obj_pos_w[0];
    pose_b_msg.Y_c = obj_pos_w[1];
    pose_b_msg.Z_c = obj_pos_w[2];
    pose_b_msg.header.stamp = pc_msg->header.stamp;
    pub_obj_body.publish(pose_b_msg);

    obj_cal_pose.pose.position.x = pose_b_msg.X_c;
    obj_cal_pose.pose.position.y = pose_b_msg.Y_c;
    obj_cal_pose.pose.position.z = pose_b_msg.Z_c;
    // obj_gt_path.header.stamp = lidar->header.stamp;
    obj_cal_path.poses.push_back(obj_cal_pose);
    obj_cal_path.header.frame_id = "world";
    pub_obj_cal.publish(obj_cal_path);
}

void gta_ns::TrackerNodeletClass::gt_cb(const geometry_msgs::PoseStamped::ConstPtr &gt)
{
    obj_gt_pose.pose = gt->pose;
    obj_gt_path.poses.push_back(obj_gt_pose);
    obj_gt_path.header.frame_id = "world";
    pub_obj_gt.publish(obj_gt_path);
}


